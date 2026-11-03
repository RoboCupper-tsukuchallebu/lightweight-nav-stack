#include <gtest/gtest.h>

#include "lightnav_obstacle_monitor/freshness.hpp"

namespace lightnav
{

using Names = std::vector<std::string>;

TEST(FreshnessTest, RecentDataIsNotStale)
{
  EXPECT_TRUE(findStaleSources("points", "velodyne", 9.8, false, 10.0, 0.5).empty());
}

TEST(FreshnessTest, ExactlyAtTimeoutIsStillFresh)
{
  EXPECT_TRUE(findStaleSources("points", "velodyne", 9.5, false, 10.0, 0.5).empty());
}

TEST(FreshnessTest, OldDataIsStale)
{
  EXPECT_EQ(findStaleSources("points", "velodyne", 9.4, false, 10.0, 0.5), Names{"points"});
}

TEST(FreshnessTest, NoDataYetIsStale)
{
  EXPECT_EQ(findStaleSources("scan", "", std::nullopt, false, 10.0, 0.5), Names{"scan"});
}

TEST(FreshnessTest, TransformFailureIsReportedSeparately)
{
  EXPECT_EQ(
    findStaleSources("points", "velodyne", 9.9, true, 10.0, 0.5), Names{"tf:velodyne"});
}

TEST(FreshnessTest, TransformFailureAndOldDataReportBoth)
{
  EXPECT_EQ(
    findStaleSources("points", "velodyne", 1.0, true, 10.0, 0.5),
    (Names{"tf:velodyne", "points"}));
}

}  // namespace lightnav
