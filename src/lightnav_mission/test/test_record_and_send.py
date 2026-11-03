import os
import pathlib

import yaml
from lightnav_mission.record_waypoints import build_argument_parser, validate_arguments
from lightnav_mission.send_waypoints import main as send_main
from lightnav_mission.waypoint_files import (
    write_yaml_atomically,
    write_yaml_with_fallback,
)


def _arguments(tmp_path, *extra):
    return build_argument_parser().parse_args(["--output", str(tmp_path / "raw.yaml"), *extra])


def test_invalid_spacing_is_rejected_before_driving(tmp_path):
    assert validate_arguments(_arguments(tmp_path, "--spacing-m", "0"))
    assert validate_arguments(_arguments(tmp_path, "--spacing-m", "nan"))
    assert validate_arguments(_arguments(tmp_path)) == ""


def test_existing_output_requires_overwrite(tmp_path):
    (tmp_path / "raw.yaml").write_text("x: 1\n")
    assert validate_arguments(_arguments(tmp_path))
    assert validate_arguments(_arguments(tmp_path, "--overwrite")) == ""


def test_atomic_write_leaves_no_temporary_file_and_replaces_content(tmp_path):
    target = tmp_path / "out.yaml"
    write_yaml_atomically(str(target), {"a": 1})
    assert yaml.safe_load(target.read_text()) == {"a": 1}
    assert os.listdir(tmp_path) == ["out.yaml"]


def test_unwritable_destination_falls_back_and_reports_location(tmp_path):
    unwritable = str(tmp_path / "missing_directory" / "raw.yaml")
    written_path = write_yaml_with_fallback(unwritable, {"a": 1})
    assert written_path != unwritable
    assert yaml.safe_load(pathlib.Path(written_path).read_text()) == {"a": 1}


def test_send_waypoints_rejects_invalid_yaml_with_nonzero_exit(tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text("waypoints: []\n")
    assert send_main([str(bad)]) != 0
    assert send_main([str(tmp_path / "missing.yaml")]) != 0
