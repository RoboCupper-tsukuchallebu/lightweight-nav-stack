"""docker/config/robot.yaml and localization_indoor.yaml carry the values of Issue #93 section 4.1.

The gtest of the node passes values through parameter_overrides, so it cannot notice a misspelled key or a value that
was changed in a params file. Parameters that the table of section 4.1 says "not written" must stay absent, so that
the code default applies (indoors: no wheel yaw weight, no GNSS, no loop closure).
"""

from pathlib import Path

import pytest
import yaml

# <repository>/src/lightnav_localization/test/<this file>
CONFIG_DIRECTORY = Path(__file__).resolve().parents[3] / "docker" / "config"
ROBOT_PARAMS_PATH = CONFIG_DIRECTORY / "robot.yaml"
INDOOR_PARAMS_PATH = CONFIG_DIRECTORY / "localization_indoor.yaml"

ROBOT_EXPECTED_VALUES = {
    "absolute_sigma_tilt_rad": 0.3,
    "absolute_sigma_z_m": 2.0,
    "wheel_sigma_yaw_per_m": 0.2,
    "gps_min_status": 0,
    "gps_topic": "/fix_all",
    "init_gps_max_std_m": 3.0,
    "init_gps_reported_std_max_m": 10.0,
    "loop_closure_enabled": True,
}
INDOOR_EXPECTED_VALUES = {
    "absolute_sigma_tilt_rad": 0.3,
    "absolute_sigma_z_m": 2.0,
}
INDOOR_PARAMETERS_THAT_MUST_BE_ABSENT = (
    "wheel_sigma_yaw_per_m",
    "gps_min_status",
    "init_gps_max_std_m",
    "init_gps_reported_std_max_m",
    "loop_closure_enabled",
)
# Section 4.1: the code default stays for these in both files.
PARAMETERS_THAT_MUST_BE_ABSENT_IN_BOTH_FILES = (
    "lidar_odom_gate_yaw_rad",
    "loop_closure_min_path_gap_m",
    "loop_closure_query_spacing_m",
    "loop_closure_search_radius_m",
    "loop_closure_search_radius_per_m",
    "loop_closure_max_search_radius_m",
    "loop_closure_max_rounds",
    "map_save_keyframe_clouds",
)


def read_localization_parameters(params_path):
    with params_path.open() as params_file:
        return yaml.safe_load(params_file)["localization_node"]["ros__parameters"]


@pytest.mark.parametrize("name", sorted(ROBOT_EXPECTED_VALUES))
def test_robot_yaml_has_the_value_of_the_issue(name):
    parameters = read_localization_parameters(ROBOT_PARAMS_PATH)
    assert name in parameters, f"{name} is missing from robot.yaml"
    assert parameters[name] == ROBOT_EXPECTED_VALUES[name]
    # YAML 0 / 1 and true / false are different types for the node: a bool must not stand in for a number.
    assert type(parameters[name]) is type(ROBOT_EXPECTED_VALUES[name])


@pytest.mark.parametrize("name", sorted(INDOOR_EXPECTED_VALUES))
def test_indoor_yaml_has_the_tilt_and_height_values(name):
    parameters = read_localization_parameters(INDOOR_PARAMS_PATH)
    assert name in parameters, f"{name} is missing from localization_indoor.yaml"
    assert parameters[name] == INDOOR_EXPECTED_VALUES[name]
    assert type(parameters[name]) is type(INDOOR_EXPECTED_VALUES[name])


@pytest.mark.parametrize("name", INDOOR_PARAMETERS_THAT_MUST_BE_ABSENT)
def test_indoor_yaml_keeps_the_code_default_of_the_outdoor_only_parameters(name):
    assert name not in read_localization_parameters(INDOOR_PARAMS_PATH)


@pytest.mark.parametrize("params_path", [ROBOT_PARAMS_PATH, INDOOR_PARAMS_PATH], ids=lambda path: path.name)
@pytest.mark.parametrize("name", PARAMETERS_THAT_MUST_BE_ABSENT_IN_BOTH_FILES)
def test_parameters_that_keep_the_code_default_are_not_written(params_path, name):
    assert name not in read_localization_parameters(params_path)
