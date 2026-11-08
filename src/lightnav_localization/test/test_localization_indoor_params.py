"""docker/config/localization_indoor.yaml keeps GNSS off (Issue #43 section 12).

The gtest of the node passes the same values through parameter_overrides, so it cannot notice a misspelled key or a
gps_topic that was changed to a topic somebody publishes (a usable fix would then silently become the GNSS origin).
"""

from pathlib import Path

import yaml

# <repository>/src/lightnav_localization/test/<this file>
INDOOR_PARAMS_PATH = Path(__file__).resolve().parents[3] / "docker" / "config" / "localization_indoor.yaml"
GNSS_TOPICS_IN_USE = ("/gps/fix", "/rtk/fix_all")


def read_indoor_parameters():
    with INDOOR_PARAMS_PATH.open() as params_file:
        return yaml.safe_load(params_file)["localization_node"]["ros__parameters"]


def test_gps_topic_is_a_topic_nobody_publishes():
    gps_topic = read_indoor_parameters()["gps_topic"]
    assert gps_topic == "/lightnav/gnss_disabled"
    assert gps_topic not in GNSS_TOPICS_IN_USE


def test_mapping_may_start_without_gps_in_the_shared_map_directory():
    parameters = read_indoor_parameters()
    assert parameters["allow_mapping_without_gps"] is True
    assert parameters["map_output_dir"] == "/data/map"
    assert parameters["base_frame"] == "base_footprint"
