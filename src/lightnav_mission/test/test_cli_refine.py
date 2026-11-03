import pytest
import yaml
from lightnav_mission.refine_waypoints import main


def _write(tmp_path, stamps):
    raw = tmp_path / "raw.yaml"
    raw.write_text(yaml.safe_dump({"frame_id": "map", "waypoints": [
        {"stamp": stamp, "x": stamp, "y": 0.0, "yaw": 0.0} for stamp in stamps]}))
    trajectory = tmp_path / "trajectory.csv"
    lines = ["stamp_sec,x,y,z,roll,pitch,yaw,position_std_m"]
    lines += [f"{t},{t + 0.3},-0.2,0,0,0,0,0.05" for t in range(11)]
    trajectory.write_text("\n".join(lines) + "\n")
    return raw, trajectory


def test_exit_zero_and_provenance(tmp_path):
    raw, trajectory = _write(tmp_path, [1.0, 5.0])
    output = tmp_path / "out.yaml"
    assert main([str(raw), str(trajectory), "--output", str(output)]) == 0
    document = yaml.safe_load(output.read_text())
    assert document["recorded_with"] == "refined"
    assert document["source_raw"] == str(raw)
    assert abs(document["waypoints"][0]["correction_m"] - 0.36) < 0.05


def test_exit_two_when_flagged_and_exit_one_on_bad_input(tmp_path):
    raw, trajectory = _write(tmp_path, [1.0, 50.0])
    output = tmp_path / "out.yaml"
    assert main([str(raw), str(trajectory), "--output", str(output)]) == 2
    assert main([str(tmp_path / "missing.yaml"), str(trajectory), "--output", str(output)]) == 1
    assert main([str(raw), str(trajectory), "--output", str(raw)]) == 1


def test_argparse_error_exits_one_not_two(tmp_path, capsys):
    raw, trajectory = _write(tmp_path, [1.0])
    for bad_argument in (["--max-correction-m", "nan"], ["--max-gap-s", "-1"], ["--unknown"]):
        with pytest.raises(SystemExit) as exit_info:
            main([str(raw), str(trajectory), *bad_argument])
        assert exit_info.value.code == 1
    capsys.readouterr()


def test_symlink_to_input_is_not_overwritten(tmp_path):
    raw, trajectory = _write(tmp_path, [1.0])
    link = tmp_path / "link.yaml"
    link.symlink_to(raw)
    before = raw.read_text()
    assert main([str(raw), str(trajectory), "--output", str(link)]) == 1
    assert raw.read_text() == before


def test_stale_flag_is_cleared_and_empty_trajectory_is_invalid_input(tmp_path):
    raw, trajectory = _write(tmp_path, [1.0])
    document = yaml.safe_load(raw.read_text())
    document["waypoints"][0].update({"flagged": "large_correction", "correction_m": 9.0})
    raw.write_text(yaml.safe_dump(document))
    output = tmp_path / "out.yaml"
    assert main([str(raw), str(trajectory), "--output", str(output)]) == 0
    assert "flagged" not in yaml.safe_load(output.read_text())["waypoints"][0]
    trajectory.write_text("stamp_sec,x,y,z,roll,pitch,yaw,position_std_m\\n")
    assert main([str(raw), str(trajectory), "--output", str(output)]) == 1


def test_short_csv_row_and_non_utf8_yaml_are_invalid_input(tmp_path):
    raw, trajectory = _write(tmp_path, [1.0])
    output = tmp_path / "out.yaml"
    trajectory.write_text("stamp_sec,x,y,z,roll,pitch,yaw,position_std_m\\n1.0,2.0\\n")
    assert main([str(raw), str(trajectory), "--output", str(output)]) == 1
    raw.write_bytes(b"\\xff\\xfe\\x00bad")
    assert main([str(raw), str(trajectory), "--output", str(output)]) == 1
