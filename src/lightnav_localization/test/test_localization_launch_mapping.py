"""localization.launch.py mode:=mapping needs no map_file, reaches ACTIVE and saves once on SIGINT."""

import launch_testing
import pytest
from localization_launch_support import (
    SHUTDOWN_TIMEOUT_SECONDS,
    LocalizationLaunchCase,
    build_launch_description,
)

OUTPUT_TIMEOUT_SECONDS = 10
MAPPING_START_TIMEOUT_SECONDS = 30.0
KEYFRAME_COLLECTION_SECONDS = 3.0
MAP_FILE_NAMES = ("map.pcd", "trajectory.csv", "map.yaml")


@pytest.mark.launch_test
def generate_test_description():
    return build_launch_description("mapping")


class TestMappingModeLaunch(LocalizationLaunchCase):
    def test_activates_and_saves_once_on_sigint(self, proc_info, proc_output, launch_process, output_directory):
        self.assertTrue(self.wait_until_active(), "localization_node did not become active")
        self.publish_lidar_extrinsic()
        started = self.publish_inputs_until(
            lambda: proc_output.waitFor("initialized mapping", process=launch_process, timeout=0.01),
            MAPPING_START_TIMEOUT_SECONDS,
        )
        self.assertTrue(started, "mapping did not start")
        self.publish_inputs_until(lambda: False, KEYFRAME_COLLECTION_SECONDS)
        self.assertFalse((output_directory / "map.yaml").exists(), "the map was written before the stop")

        self.send_sigint_to(launch_process)

        proc_info.assertWaitForShutdown(process=launch_process, timeout=SHUTDOWN_TIMEOUT_SECONDS)
        proc_output.assertWaitFor(
            "mapping map saved on shutdown", process=launch_process, timeout=OUTPUT_TIMEOUT_SECONDS
        )
        for name in MAP_FILE_NAMES:
            self.assertTrue((output_directory / name).is_file(), f"{name} was not written")
        saved_count = sum(
            output.text.decode(errors="replace").count("mapping map saved on shutdown") for output in proc_output
        )
        self.assertEqual(saved_count, 1)
