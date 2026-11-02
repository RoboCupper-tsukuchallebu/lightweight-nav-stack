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
