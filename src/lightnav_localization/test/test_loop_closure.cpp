#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <cstdio>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdexcept>

#include "lightnav_localization/map_builder.hpp"
#include "lightnav_localization/matcher.hpp"
#include "lightnav_localization/localizer.hpp"
#include "lightnav_localization/map_metadata.hpp"

#include "lightnav_localization/loop_closure.hpp"

using namespace lightnav_localization;

namespace {
BatchPose pose_at(double position_x, double position_y) {
  BatchPose pose;
  pose.map_to_base.translation() = Eigen::Vector3d(position_x, position_y, 0.0);
  return pose;
}
}

TEST(LoopClosure, CandidatesAreOrderedByTravelAndThinnedByPathLength) {
  std::vector<BatchPose> poses;
  for (int index = 0; index <= 20; ++index) poses.push_back(pose_at(index * 0.5, 0.0));
  for (int index = 1; index <= 20; ++index) poses.push_back(pose_at(10.0 - index * 0.5, 0.0));
  const std::vector<double> lengths = loop_path_lengths(poses);

  const auto candidates = select_loop_candidates(poses, lengths, LoopClosureParameters{});

  ASSERT_FALSE(candidates.empty());
  EXPECT_EQ(candidates.front().newer_index, poses.size() - 1);
  EXPECT_EQ(candidates.front().older_index, 0u);
  for (std::size_t index = 1; index < candidates.size(); ++index) {
    EXPECT_GE(candidates[index - 1].travel_m, candidates[index].travel_m);
  }
}

TEST(LoopClosure, SupportRequiresIndependentSourcesButCanShareTarget) {
  std::vector<BatchPose> poses{pose_at(0.0, 0.0), pose_at(0.2, 0.0), pose_at(1.4, 0.0)};
  const std::vector<double> lengths{0.0, 10.0, 11.2};
  VerifiedLoop first;
  first.candidate = {0, 1, 10.0};
  first.older_to_newer = poses[0].map_to_base.inverse() * poses[1].map_to_base;
  VerifiedLoop second;
  second.candidate = {0, 2, 11.2};
  second.older_to_newer = poses[0].map_to_base.inverse() * poses[2].map_to_base;

  EXPECT_TRUE(supported_loops({first}, poses, lengths, LoopClosureParameters{}).empty());
  EXPECT_EQ(supported_loops({first, second}, poses, lengths, LoopClosureParameters{}).size(), 2u);
  second.older_to_newer.translation().y() += 1.0;
  EXPECT_TRUE(supported_loops({first, second}, poses, lengths, LoopClosureParameters{}).empty());
}

TEST(LoopClosure, IterationLimitReservesSafetyMarginAndHonorsBatchLimit) {
  EXPECT_EQ(loop_iteration_limit(9.0, 1.0, 100), 6);
  EXPECT_EQ(loop_iteration_limit(0.5, 1.0, 100), 0);
  EXPECT_EQ(loop_iteration_limit(1000.0, 1.0, 100), 100);
}

TEST(LoopClosure, ClusteredMatchesDoNotSupportOneAnother) {
  // This injection is after candidate thinning, to exercise clustered verification results (L7).
  std::vector<BatchPose> poses{pose_at(0.0, 0.0)};
  std::vector<double> lengths{0.0};
  std::vector<VerifiedLoop> loops;
  for (int index = 0; index < 5; ++index) {
    poses.push_back(pose_at(index * 0.2, 0.0));
    lengths.push_back(10.0 + index * 0.2);
    VerifiedLoop loop;
    loop.candidate = {0, static_cast<std::size_t>(index + 1), lengths.back()};
    loop.older_to_newer = poses.back().map_to_base;
    loops.push_back(loop);
  }

  EXPECT_TRUE(supported_loops(loops, poses, lengths, LoopClosureParameters{}).empty());
}

namespace {
struct SyntheticLoopGraph {
  PoseGraph graph;
  std::vector<KeyframeCloud> keyframes;
  std::vector<BatchPose> poses;
  SyntheticLoopGraph() : graph([] { PoseGraphParameters parameters; parameters.record_history = true; return parameters; }()) {
    graph.add_first_node(0.0, Eigen::Isometry3d::Identity(), 0.1, 0.1);
    keyframes.push_back({0, 0.0, {}});
    for (int index = 1; index <= 20; ++index) {
      const double position_x = index <= 10 ? static_cast<double>(index) : static_cast<double>(20 - index);
      const double previous_x = index <= 11 ? static_cast<double>(index - 1)
                                             : static_cast<double>(21 - index);
      Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
      relative.translation().x() = position_x - previous_x;
      Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
      guess.translation().x() = position_x;
      const int node = graph.add_node_with_wheel_factor(index, guess, relative);
      keyframes.push_back({node, static_cast<double>(index), {}});
    }
    EXPECT_TRUE(graph.batch_optimize(100));
    poses = graph.batch_poses();
  }
  std::optional<VerifiedLoop> match(const LoopCandidate& candidate) const {
    VerifiedLoop loop;
    loop.candidate = candidate;
    loop.older_to_newer = poses[candidate.older_index].map_to_base.inverse() *
                          poses[candidate.newer_index].map_to_base;
    loop.information = diagonal_information(0.05, 0.01);
    return loop;
  }
};
}

TEST(LoopClosure, BatchExceptionRestoresPassZeroAndRemovesFactors) {
  SyntheticLoopGraph fixture;
  LoopClosureTestHooks hooks;
  hooks.verify = [&](const LoopCandidate& candidate) { return fixture.match(candidate); };
  hooks.elapsed_seconds = [] { return 0.0; };
  hooks.before_batch = [](int) { throw std::runtime_error("injected failure"); };

  const LoopClosureResult result = close_loops(fixture.graph, fixture.keyframes, {}, 100, 0.05, 0.0, &hooks);

  EXPECT_EQ(result.count, 0);
  EXPECT_NE(result.warning.find("loop closure skipped:"), std::string::npos);
  ASSERT_EQ(result.poses.size(), fixture.poses.size());
  for (std::size_t index = 0; index < result.poses.size(); ++index) {
    EXPECT_TRUE(result.poses[index].map_to_base.matrix().isApprox(fixture.poses[index].map_to_base.matrix()));
  }
  EXPECT_TRUE(fixture.graph.loop_factor_chi_squared().empty());
}

TEST(LoopClosure, DeadlineDeterminesBatchIterationLimit) {
  SyntheticLoopGraph fixture;
  LoopClosureTestHooks hooks;
  hooks.verify = [&](const LoopCandidate& candidate) { return fixture.match(candidate); };
  hooks.elapsed_seconds = [] { return 20.0; };
  hooks.iteration_seconds = 1.0;
  int recorded_limit = 0;
  hooks.before_batch = [&](int limit) { recorded_limit = limit; throw std::runtime_error("stop"); };

  close_loops(fixture.graph, fixture.keyframes, {}, 100, 0.05, 0.0, &hooks);

  const double remaining = 5.0 - 1.5 * fixture.graph.batch_marginals_seconds();
  EXPECT_EQ(recorded_limit, loop_iteration_limit(remaining, 1.0, 100));
  EXPECT_GT(recorded_limit, 0);
}

TEST(LoopClosure, DisabledAndExpiredBudgetKeepPassZero) {
  SyntheticLoopGraph fixture;
  LoopClosureParameters disabled;
  disabled.enabled = false;
  const auto disabled_result = close_loops(fixture.graph, fixture.keyframes, disabled, 100, 0.05, 0.0);
  LoopClosureParameters expired;
  expired.time_budget_s = 0.0;
  const auto expired_result = close_loops(fixture.graph, fixture.keyframes, expired, 100, 0.05, 0.0);

  EXPECT_TRUE(disabled_result.warning.empty());
  EXPECT_EQ(expired_result.warning, "loop closure skipped: time budget");
  EXPECT_EQ(expired_result.count, 0);
  ASSERT_EQ(disabled_result.poses.size(), expired_result.poses.size());
  for (std::size_t index = 0; index < disabled_result.poses.size(); ++index) {
    EXPECT_LT((disabled_result.poses[index].map_to_base.matrix() -
               expired_result.poses[index].map_to_base.matrix()).norm(), 1e-9);
  }
  EXPECT_TRUE(fixture.graph.loop_factor_chi_squared().empty());
}

TEST(LoopClosure, GpsUncertaintyTightensTheTranslationGate) {
  std::vector<BatchPose> poses{pose_at(0.0, 0.0), pose_at(1.0, 0.0)};
  poses[0].index = 0;
  poses[1].index = 1;
  poses[0].position_and_yaw_std = Eigen::Vector3d(0.3, 0.3, 0.1);
  poses[1].position_and_yaw_std = Eigen::Vector3d(0.3, 0.3, 0.1);
  const LoopCandidate candidate{0, 1, 100.0};
  const std::vector<double> lengths{0.0, 100.0};

  EXPECT_LE(loop_translation_gate(candidate, poses, lengths, {0, 1}, {}), 2.0);
  EXPECT_DOUBLE_EQ(loop_translation_gate(candidate, poses, lengths, {}, {}), 10.5);
  poses[1].position_and_yaw_std.reset();
  EXPECT_DOUBLE_EQ(loop_translation_gate(candidate, poses, lengths, {0, 1}, {}), 10.5);
}

TEST(LoopClosure, StraightCourseDoesNotProduceRevisitCandidates) {
  std::vector<BatchPose> poses;
  for (int index = 0; index <= 200; ++index) poses.push_back(pose_at(index, 0.0));
  const auto lengths = loop_path_lengths(poses);

  EXPECT_TRUE(select_loop_candidates(poses, lengths, {}).empty());
}

TEST(LoopClosure, AConflictingLoopIsRemovedUsingItsWhitenedResidual) {
  SyntheticLoopGraph fixture;
  LoopClosureTestHooks hooks;
  hooks.verify = [&](const LoopCandidate& candidate) { return fixture.match(candidate); };
  hooks.elapsed_seconds = [] { return 0.0; };
  hooks.iteration_seconds = 0.001;
  std::vector<VerifiedLoop> correct_loops;
  const auto lengths = loop_path_lengths(fixture.poses);
  for (const LoopCandidate& candidate : select_loop_candidates(fixture.poses, lengths, {})) {
    correct_loops.push_back(*fixture.match(candidate));
  }
  const std::size_t correct_count = supported_loops(correct_loops, fixture.poses, lengths, {}).size();
  bool injected = false;
  hooks.before_batch = [&](int) {
    if (injected) return;
    Eigen::Isometry3d incorrect = Eigen::Isometry3d::Identity();
    incorrect.translation().x() = 5.0;
    ASSERT_TRUE(fixture.graph.add_loop_factor(0, 20, incorrect, diagonal_information(0.05, 0.01)));
    injected = true;
  };

  const auto result = close_loops(fixture.graph, fixture.keyframes, {}, 100, 0.05, 0.0, &hooks);

  EXPECT_TRUE(result.warning.empty()) << result.warning;
  EXPECT_NE(result.statistics.find("removed chi2="), std::string::npos);
  EXPECT_EQ(static_cast<std::size_t>(result.count), correct_count);
  const std::size_t first_removal = result.statistics.find("removed chi2=");
  EXPECT_EQ(result.statistics.find("removed chi2=", first_removal + 1), std::string::npos);
  ASSERT_EQ(result.poses.size(), fixture.poses.size());
  EXPECT_LT((result.poses.back().map_to_base.translation() -
             fixture.poses.back().map_to_base.translation()).norm(), 0.3);
  EXPECT_TRUE(fixture.graph.loop_factor_chi_squared().empty());
}

TEST(LoopClosure, StraightCorridorMatchIsRejectedForDegeneracy) {
  PointList corridor;
  for (int along = -40; along <= 40; ++along) {
    for (int height = 0; height < 8; ++height) {
      corridor.emplace_back(along * 0.25f, -2.0f, height * 0.25f);
      corridor.emplace_back(along * 0.25f, 2.0f, height * 0.25f);
    }
  }
  PointList shifted = corridor;
  for (Eigen::Vector3f& point : shifted) point.x() -= 5.0f;
  std::vector<KeyframeCloud> keyframes{{0, 0.0, corridor}, {1, 1.0, shifted}};
  std::vector<BatchPose> poses{pose_at(0.0, 0.0), pose_at(0.5, 0.0)};
  poses[0].index = 0;
  poses[1].index = 1;
  const LoopCandidate candidate{0, 1, 8.0};

  MatcherParameters matcher_parameters;
  matcher_parameters.max_correspondence_distance_m = 0.5;
  const MatchResult match = match_scan(PreparedTarget(corridor), shifted,
                                        Eigen::Isometry3d::Identity(), matcher_parameters);
  const Eigen::Matrix3d translation = match.information.bottomRightCorner<3, 3>();
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenvalues(translation);
  const auto result = verify_loop_candidate(candidate, poses, keyframes, {0.0, 8.0}, {}, {}, 0.05);

  ASSERT_TRUE(match.converged);
  ASSERT_GT(eigenvalues.eigenvalues()[2], 0.0);
  EXPECT_LT(eigenvalues.eigenvalues()[0] / eigenvalues.eigenvalues()[2], 0.05);
  EXPECT_FALSE(result.has_value());
}

TEST(LoopClosure, CovarianceBudgetKeepsClosedPosesWithUnknownUncertainty) {
  SyntheticLoopGraph fixture;
  LoopClosureTestHooks hooks;
  hooks.verify = [&](const LoopCandidate& candidate) { return fixture.match(candidate); };
  bool batch_started = false;
  hooks.elapsed_seconds = [&] { return batch_started ? 25.0 : 0.0; };
  hooks.iteration_seconds = 0.001;
  hooks.before_batch = [&](int) { batch_started = true; };

  const LoopClosureResult result = close_loops(fixture.graph, fixture.keyframes, {}, 100, 0.05, 0.0, &hooks);

  EXPECT_GE(result.count, 2);
  EXPECT_EQ(result.warning, "covariance skipped: time budget");
  ASSERT_FALSE(result.poses.empty());
  for (const BatchPose& pose : result.poses) EXPECT_FALSE(pose.position_and_yaw_std.has_value());
}

TEST(LoopClosure, ADeadlineTooShortForOneIterationSkipsBatch) {
  SyntheticLoopGraph fixture;
  LoopClosureTestHooks hooks;
  hooks.verify = [&](const LoopCandidate& candidate) { return fixture.match(candidate); };
  hooks.elapsed_seconds = [] { return 24.9; };
  hooks.iteration_seconds = 1.0;
  bool batch_called = false;
  hooks.before_batch = [&](int) { batch_called = true; };

  const auto result = close_loops(fixture.graph, fixture.keyframes, {}, 100, 0.05, 0.0, &hooks);

  EXPECT_FALSE(batch_called);
  EXPECT_EQ(result.count, 0);
  EXPECT_EQ(result.warning, "loop closure skipped: time budget");
}

TEST(LoopClosure, VerificationLimitStartsWithTheLongestRevisit) {
  PoseGraphParameters graph_parameters;
  graph_parameters.record_history = true;
  PoseGraph graph(graph_parameters);
  graph.add_first_node(0.0, Eigen::Isometry3d::Identity(), 0.1, 0.1);
  std::vector<KeyframeCloud> keyframes{{0, 0.0, {}}};
  for (int index = 1; index <= 450; ++index) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().x() = index % 2;
    Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
    motion.translation().x() = index % 2 ? 1.0 : -1.0;
    const int node = graph.add_node_with_wheel_factor(index, pose, motion);
    keyframes.push_back({node, static_cast<double>(index), {}});
  }
  ASSERT_TRUE(graph.batch_optimize(100));
  std::vector<LoopCandidate> checked;
  LoopClosureTestHooks hooks;
  hooks.elapsed_seconds = [] { return 0.0; };
  hooks.verify = [&](const LoopCandidate& candidate) -> std::optional<VerifiedLoop> {
    checked.push_back(candidate);
    return std::nullopt;
  };

  close_loops(graph, keyframes, {}, 100, 0.05, 0.0, &hooks);

  ASSERT_EQ(checked.size(), 200u);
  EXPECT_EQ(checked.front().newer_index, 450u);
  EXPECT_EQ(checked.front().older_index, 0u);
}

TEST(LoopClosure, DeadlineIterationLimitDoesNotUseUnconvergedResult) {
  SyntheticLoopGraph fixture;
  LoopClosureTestHooks hooks;
  hooks.verify = [&](const LoopCandidate& candidate) {
    auto loop = fixture.match(candidate);
    loop->older_to_newer.translation().x() += 0.1;
    return loop;
  };
  hooks.elapsed_seconds = [] { return 0.0; };
  hooks.iteration_seconds = 16.0;
  int limit = 0;
  hooks.before_batch = [&](int value) { limit = value; };

  const auto result = close_loops(fixture.graph, fixture.keyframes, {}, 100, 0.05, 0.0, &hooks);

  EXPECT_EQ(limit, 1);
  EXPECT_EQ(result.count, 0);
  EXPECT_EQ(result.warning, "loop closure skipped: time budget");
  ASSERT_EQ(result.poses.size(), fixture.poses.size());
  for (std::size_t index = 0; index < result.poses.size(); ++index) {
    EXPECT_LT((result.poses[index].map_to_base.matrix() - fixture.poses[index].map_to_base.matrix()).norm(), 1e-9);
  }
}

TEST(LoopClosure, BatchIterationLimitReportsNotConverged) {
  SyntheticLoopGraph fixture;
  LoopClosureTestHooks hooks;
  hooks.verify = [&](const LoopCandidate& candidate) {
    auto loop = fixture.match(candidate);
    loop->older_to_newer.translation().x() += 0.1;
    return loop;
  };
  hooks.elapsed_seconds = [] { return 0.0; };
  hooks.iteration_seconds = 0.001;

  const auto result = close_loops(fixture.graph, fixture.keyframes, {}, 1, 0.05, 0.0, &hooks);

  EXPECT_EQ(result.count, 0);
  EXPECT_EQ(result.warning, "loop closure skipped: not converged");
}

TEST(LoopClosure, DisabledProducesTheSamePosesAsDefaultBatchOptimization) {
  SyntheticLoopGraph disabled_graph;
  SyntheticLoopGraph baseline_graph;
  LoopClosureParameters parameters;
  parameters.enabled = false;
  int verification_calls = 0;
  int additional_batch_calls = 0;
  LoopClosureTestHooks hooks;
  hooks.verify = [&](const LoopCandidate&) -> std::optional<VerifiedLoop> {
    ++verification_calls;
    return std::nullopt;
  };
  hooks.before_batch = [&](int) { ++additional_batch_calls; };

  const auto result = close_loops(disabled_graph.graph, disabled_graph.keyframes, parameters,
                                  100, 0.05, 0.0, &hooks);

  EXPECT_EQ(verification_calls, 0);
  EXPECT_EQ(additional_batch_calls, 0);
  EXPECT_EQ(result.count, 0);
  ASSERT_EQ(result.poses.size(), baseline_graph.poses.size());
  for (std::size_t index = 0; index < result.poses.size(); ++index) {
    EXPECT_LT((result.poses[index].map_to_base.matrix() -
               baseline_graph.poses[index].map_to_base.matrix()).norm(), 1e-9);
  }
}

TEST(LoopClosure, SaveMapFallsBackToAllPassZeroFilesAfterBatchFailure) {
  LocalizerParameters localizer_parameters;
  localizer_parameters.mapping = true;
  Localizer localizer(localizer_parameters, {}, std::nullopt);
  std::vector<Eigen::Isometry3d> poses;
  for (int index = 0; index <= 20; ++index) {
    poses.push_back(pose_at(index <= 10 ? index : 20 - index, 0.0).map_to_base);
  }
  localizer.seed_mapping_for_test(poses);
  const std::filesystem::path base = std::filesystem::path(::testing::TempDir()) / "loop_fallback_baseline";
  const std::filesystem::path fallback = std::filesystem::path(::testing::TempDir()) / "loop_fallback_failed";
  MapOutputParameters parameters;
  parameters.map_output_dir = base.string();
  parameters.loop_closure.enabled = false;
  MapOutputResult baseline_result;
  std::string error;
  ASSERT_TRUE(localizer.save_map(parameters, baseline_result, error)) << error;
  LoopClosureTestHooks hooks;
  hooks.verify = [&](const LoopCandidate& candidate) -> std::optional<VerifiedLoop> {
    VerifiedLoop loop;
    loop.candidate = candidate;
    loop.older_to_newer = poses[candidate.older_index].inverse() * poses[candidate.newer_index];
    loop.information = diagonal_information(0.05, 0.01);
    return loop;
  };
  hooks.elapsed_seconds = [] { return 0.0; };
  hooks.before_batch = [](int) { throw std::runtime_error("injected batch failure"); };
  parameters.map_output_dir = fallback.string();
  parameters.loop_closure.enabled = true;
  parameters.loop_closure_test_hooks = &hooks;
  MapOutputResult failed_result;

  ASSERT_TRUE(localizer.save_map(parameters, failed_result, error)) << error;

  EXPECT_NE(failed_result.warning.find("loop closure skipped:"), std::string::npos);
  EXPECT_EQ(read_map_metadata((fallback / "map.yaml").string()).loop_closure_count, 0);
  for (const char* filename : {"map.pcd", "map.yaml", "trajectory.csv"}) {
    ASSERT_TRUE(std::filesystem::exists(fallback / filename));
    std::ifstream baseline_stream(base / filename, std::ios::binary);
    std::ifstream fallback_stream(fallback / filename, std::ios::binary);
    const std::string baseline(std::istreambuf_iterator<char>(baseline_stream),
                               std::istreambuf_iterator<char>{});
    const std::string actual(std::istreambuf_iterator<char>(fallback_stream),
                             std::istreambuf_iterator<char>{});
    EXPECT_EQ(actual, baseline) << filename;
  }
  std::filesystem::remove_all(base);
  std::filesystem::remove_all(fallback);
}

TEST(LoopClosure, ReportsSaveTimeForTwoThousandSyntheticKeyframes) {
  LocalizerParameters localizer_parameters;
  localizer_parameters.mapping = true;
  Localizer localizer(localizer_parameters, {}, std::nullopt);
  std::vector<Eigen::Isometry3d> poses;
  for (int index = 0; index < 2000; ++index) poses.push_back(pose_at(index * 0.2, 0.0).map_to_base);
  localizer.seed_mapping_for_test(poses);
  const std::filesystem::path output = std::filesystem::path(::testing::TempDir()) / "loop_2000_keyframes";
  MapOutputParameters parameters;
  parameters.map_output_dir = output.string();
  MapOutputResult result;
  std::string error;
  const auto start = std::chrono::steady_clock::now();

  ASSERT_TRUE(localizer.save_map(parameters, result, error)) << error;

  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_EQ(result.keyframe_count, 2000u);
  std::printf("[ LOOP_A6_2000 ] save_s=%.6f %s\n", seconds, result.statistics.c_str());
  std::filesystem::remove_all(output);
}

TEST(LoopClosure, DisabledSaveMatchesIndependentDefaultBatchOptimization) {
  LocalizerParameters localizer_parameters;
  localizer_parameters.mapping = true;
  Localizer saved(localizer_parameters, {}, std::nullopt);
  Localizer baseline(localizer_parameters, {}, std::nullopt);
  std::vector<Eigen::Isometry3d> poses;
  for (int index = 0; index <= 20; ++index) poses.push_back(pose_at(index * 0.5, 0.0).map_to_base);
  saved.seed_mapping_for_test(poses);
  baseline.seed_mapping_for_test(poses);
  const std::vector<BatchPose> expected = baseline.batch_poses_for_test(100);
  const std::filesystem::path output = std::filesystem::path(::testing::TempDir()) / "loop_disabled_regression";
  MapOutputParameters parameters;
  parameters.map_output_dir = output.string();
  parameters.loop_closure.enabled = false;
  int verification_calls = 0;
  int additional_batch_calls = 0;
  int add_factor_calls = 0;
  LoopClosureTestHooks hooks;
  hooks.verify = [&](const LoopCandidate&) -> std::optional<VerifiedLoop> {
    ++verification_calls;
    return std::nullopt;
  };
  hooks.before_batch = [&](int) { ++additional_batch_calls; };
  hooks.before_add_factor = [&] { ++add_factor_calls; };
  parameters.loop_closure_test_hooks = &hooks;
  MapOutputResult result;
  std::string error;

  ASSERT_TRUE(saved.save_map(parameters, result, error)) << error;

  EXPECT_EQ(verification_calls, 0);
  EXPECT_EQ(additional_batch_calls, 0);
  EXPECT_EQ(add_factor_calls, 0);
  std::ifstream trajectory(output / "trajectory.csv");
  std::string line;
  std::getline(trajectory, line);
  std::size_t index = 0;
  while (std::getline(trajectory, line)) {
    ASSERT_LT(index, expected.size());
    std::istringstream row(line);
    std::string field;
    std::getline(row, field, ',');
    std::getline(row, field, ',');
    EXPECT_NEAR(std::stod(field), expected[index].map_to_base.translation().x(), 1e-9);
    std::getline(row, field, ',');
    EXPECT_NEAR(std::stod(field), expected[index].map_to_base.translation().y(), 1e-9);
    for (int component = 0; component < 4; ++component) {
      std::getline(row, field, ',');
      EXPECT_NEAR(std::stod(field), 0.0, 1e-9);
    }
    ++index;
  }
  EXPECT_EQ(index, expected.size());
  std::filesystem::remove_all(output);
}

TEST(LoopClosure, GpsGateRejectsAThreeMeterFalseMatch) {
  std::vector<BatchPose> poses{pose_at(0.0, 0.0), pose_at(0.0, 0.0)};
  poses[0].index = 0;
  poses[1].index = 1;
  poses[0].position_and_yaw_std = Eigen::Vector3d(0.3, 0.3, 0.1);
  poses[1].position_and_yaw_std = Eigen::Vector3d(0.3, 0.3, 0.1);
  const LoopCandidate candidate{0, 1, 100.0};
  const std::vector<double> lengths{0.0, 100.0};
  MatchResult match;
  match.converged = true;
  match.inlier_ratio = 1.0;
  match.information = Eigen::Matrix<double, 6, 6>::Identity();
  match.target_to_source.translation().x() = 3.0;
  LoopVerificationFailure failure = LoopVerificationFailure::kFine;

  const auto in_gps = assess_loop_match(candidate, match, poses, lengths, {0, 1}, {}, 0.05, &failure);
  const auto without_gps = assess_loop_match(candidate, match, poses, lengths, {}, {}, 0.05);

  EXPECT_FALSE(in_gps.has_value());
  EXPECT_EQ(failure, LoopVerificationFailure::kCorrectionGate);
  EXPECT_TRUE(without_gps.has_value());
}

TEST(LoopClosure, DriftGateRejectsAdjacentPeriodAtOneThirdDrift) {
  const std::vector<BatchPose> poses{pose_at(0.0, 0.0), pose_at(0.0, 0.0)};
  const LoopCandidate candidate{0, 1, 8.0};
  const std::vector<double> lengths{0.0, 8.0};
  MatchResult match;
  match.converged = true;
  match.inlier_ratio = 1.0;
  match.information = Eigen::Matrix<double, 6, 6>::Identity();
  match.target_to_source.translation().x() = 1.0;

  const auto correct_period = assess_loop_match(candidate, match, poses, lengths, {}, {}, 0.05);
  match.target_to_source.translation().x() = -2.0;
  const auto adjacent_period = assess_loop_match(candidate, match, poses, lengths, {}, {}, 0.05);
  match.target_to_source.translation().x() = 1.5;
  const auto half_period = assess_loop_match(candidate, match, poses, lengths, {}, {}, 0.05);
  std::printf("[ LOOP_A3_HALF_PERIOD_GATE ] accepted=%d\n", half_period.has_value() ? 1 : 0);

  EXPECT_TRUE(correct_period.has_value());
  EXPECT_FALSE(adjacent_period.has_value());
}
