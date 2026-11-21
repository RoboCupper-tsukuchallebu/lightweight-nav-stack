import pytest
import yaml
from lightnav_mission.trajectory_to_waypoints import build_argument_parser, main
from lightnav_mission.waypoint_files import load_waypoints_file

TRAJECTORY_HEADER = "stamp_sec,x,y,z,roll,pitch,yaw,position_std_m"
FIRST_STAMP_SEC = 100.0


def _trajectory_line(stamp_sec, x):
    return f"{stamp_sec},{x},0.5,0,0,0,0.1,0.05"


def _write_trajectory(tmp_path, xs):
    """1 行 1 秒で +x へ進む軌跡を書く。"""
    trajectory = tmp_path / "trajectory.csv"
    lines = [TRAJECTORY_HEADER]
    lines += [_trajectory_line(FIRST_STAMP_SEC + index, x) for index, x in enumerate(xs)]
    trajectory.write_text("\n".join(lines) + "\n")
    return trajectory


def test_thins_trajectory_and_keeps_first_and_last(tmp_path):
    trajectory = _write_trajectory(tmp_path, [0, 1, 2, 3, 4, 5])
    output = tmp_path / "waypoints.yaml"

    assert main([str(trajectory), "--output", str(output)]) == 0

    document = yaml.safe_load(output.read_text())
    assert document["frame_id"] == "map"
    assert document["recorded_with"] == "trajectory"
    assert document["source_trajectory"] == str(trajectory)
    assert [waypoint["x"] for waypoint in document["waypoints"]] == [0, 2, 4, 5]
    assert document["waypoints"][1] == {"x": 2.0, "y": 0.5, "yaw": 0.1, "stamp": FIRST_STAMP_SEC + 2}


def test_output_is_readable_by_send_waypoints_loader(tmp_path):
    trajectory = _write_trajectory(tmp_path, [0, 3, 6])
    output = tmp_path / "waypoints.yaml"

    assert main([str(trajectory), "--output", str(output)]) == 0

    records = load_waypoints_file(str(output))
    assert [(record.x, record.y) for record in records] == [(0.0, 0.5), (3.0, 0.5), (6.0, 0.5)]


def test_spacing_argument_changes_the_number_of_waypoints(tmp_path):
    trajectory = _write_trajectory(tmp_path, [0, 1, 2, 3, 4, 5])
    output = tmp_path / "waypoints.yaml"

    assert main([str(trajectory), "--output", str(output), "--spacing-m", "1.0"]) == 0

    assert len(yaml.safe_load(output.read_text())["waypoints"]) == 6
    assert build_argument_parser().parse_args([str(trajectory)]).spacing_m == 2.0


def test_rows_are_used_in_stamp_order(tmp_path):
    trajectory = tmp_path / "trajectory.csv"
    shuffled_lines = [_trajectory_line(102.0, 4), _trajectory_line(100.0, 0), _trajectory_line(101.0, 2)]
    trajectory.write_text("\n".join([TRAJECTORY_HEADER, *shuffled_lines]) + "\n")
    output = tmp_path / "waypoints.yaml"

    assert main([str(trajectory), "--output", str(output)]) == 0

    assert [waypoint["x"] for waypoint in yaml.safe_load(output.read_text())["waypoints"]] == [0, 2, 4]


def test_existing_output_is_kept_unless_overwrite(tmp_path):
    trajectory = _write_trajectory(tmp_path, [0, 3])
    output = tmp_path / "waypoints.yaml"
    output.write_text("hand edited\n")

    assert main([str(trajectory), "--output", str(output)]) == 1
    assert output.read_text() == "hand edited\n"

    assert main([str(trajectory), "--output", str(output), "--overwrite"]) == 0
    assert len(yaml.safe_load(output.read_text())["waypoints"]) == 2


def test_output_pointing_at_the_input_is_rejected_even_with_overwrite(tmp_path):
    trajectory = _write_trajectory(tmp_path, [0, 3])
    link = tmp_path / "link.yaml"
    link.symlink_to(trajectory)
    before = trajectory.read_text()

    assert main([str(trajectory), "--output", str(trajectory), "--overwrite"]) == 1
    assert main([str(trajectory), "--output", str(link), "--overwrite"]) == 1
    assert trajectory.read_text() == before


@pytest.mark.parametrize("trajectory_text", [
    "stamp_sec,x,y\n1.0,2.0,3.0\n",                       # 列が足りない
    TRAJECTORY_HEADER + "\n",                              # データ行が無い
    TRAJECTORY_HEADER + "\n1.0,nan,0,0,0,0,0,0.05\n",      # 非有限値
    TRAJECTORY_HEADER + "\n1.0,2.0\n",                     # 行が短い
])
def test_invalid_trajectory_exits_one_without_writing(tmp_path, trajectory_text):
    trajectory = tmp_path / "trajectory.csv"
    trajectory.write_text(trajectory_text)
    output = tmp_path / "waypoints.yaml"

    assert main([str(trajectory), "--output", str(output)]) == 1
    assert not output.exists()


def test_missing_trajectory_and_unwritable_output_exit_one(tmp_path):
    trajectory = _write_trajectory(tmp_path, [0, 3])

    assert main([str(tmp_path / "missing.csv"), "--output", str(tmp_path / "waypoints.yaml")]) == 1
    assert main([str(trajectory), "--output", str(tmp_path / "no_such_directory" / "waypoints.yaml")]) == 1


def test_argparse_error_exits_one(tmp_path, capsys):
    trajectory = _write_trajectory(tmp_path, [0, 3])
    bad_argument_lists = (
        [str(trajectory), "--spacing-m", "0"],
        [str(trajectory), "--spacing-m", "nan"],
        [str(trajectory), "--spacing-m", "-1"],
        [str(trajectory), "--unknown"],
        [],
    )
    for bad_arguments in bad_argument_lists:
        with pytest.raises(SystemExit) as exit_info:
            main(bad_arguments)
        assert exit_info.value.code == 1
    capsys.readouterr()
