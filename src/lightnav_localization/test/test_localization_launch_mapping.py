"""localization.launch.py mode:=mapping needs no map_file, reaches ACTIVE and saves once on SIGINT."""

import unittest

import launch_testing
import pytest
from localization_launch_support import (
    OUTPUT_STREAM,
    LocalizationLaunchCase,
    build_launch_description,
    read_all_output_text,
)

MAPPING_START_TIMEOUT_SECONDS = 30.0
KEYFRAME_COLLECTION_SECONDS = 3.0
MAP_FILE_NAMES = ("map.pcd", "trajectory.csv", "map.yaml")
SAVED_ON_SHUTDOWN_MESSAGE = "mapping map saved on shutdown"


@pytest.mark.launch_test
def generate_test_description():
    return build_launch_description("mapping")


class TestMappingModeLaunch(LocalizationLaunchCase):
    def test_activates_and_receives_sigint(self, proc_output, launch_process, output_directory):
        self.assertTrue(self.wait_until_active(), "localization_node did not become active")
        self.publish_lidar_extrinsic()
        started = self.publish_inputs_until(
            lambda: proc_output.waitFor(
                "initialized mapping", process=launch_process, timeout=0.01, stream=OUTPUT_STREAM
            ),
            MAPPING_START_TIMEOUT_SECONDS,
        )
        self.assertTrue(started, "mapping did not start")
        self.publish_inputs_until(lambda: False, KEYFRAME_COLLECTION_SECONDS)
        self.assertFalse((output_directory / "map.yaml").exists(), "the map was written before the stop")
        # The exit is checked after the launch has stopped (post_shutdown_test), see the localization launch test.
        self.send_sigint_to(launch_process)


@launch_testing.post_shutdown_test()
class TestMappingModeShutdown(unittest.TestCase):
    def test_saves_the_map_once_on_shutdown(self, proc_info, proc_output, launch_process, output_directory):
        launch_testing.asserts.assertExitCodes(proc_info)
        launch_testing.asserts.assertInStream(
            proc_output, SAVED_ON_SHUTDOWN_MESSAGE, launch_process, strip_ansi_escape_sequences=True,
            stream=OUTPUT_STREAM,
        )
        for name in MAP_FILE_NAMES:
            self.assertTrue((output_directory / name).is_file(), f"{name} was not written")
        self.assertEqual(read_all_output_text(proc_output).count(SAVED_ON_SHUTDOWN_MESSAGE), 1)
