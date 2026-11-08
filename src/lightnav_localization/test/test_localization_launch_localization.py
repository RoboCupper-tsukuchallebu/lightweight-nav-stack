"""localization.launch.py mode:=localization reaches ACTIVE and SIGINT goes through on_shutdown."""

import unittest

import launch_testing
import pytest
from localization_launch_support import (
    OUTPUT_STREAM,
    OUTPUT_TIMEOUT_SECONDS,
    LocalizationLaunchCase,
    build_launch_description,
    read_all_output_text,
)


@pytest.mark.launch_test
def generate_test_description():
    return build_launch_description("localization")


class TestLocalizationModeLaunch(LocalizationLaunchCase):
    def test_activates_and_receives_sigint(self, proc_output, launch_process):
        self.assertTrue(self.wait_until_active(), "localization_node did not become active")
        proc_output.assertWaitFor(
            "localization node active in localization mode",
            process=launch_process,
            timeout=OUTPUT_TIMEOUT_SECONDS,
            stream=OUTPUT_STREAM,
        )
        # The exit is checked after the launch has stopped (post_shutdown_test): an exit inside an active test races
        # with launch_testing noticing that the processes under test have stopped.
        self.send_sigint_to(launch_process)


@launch_testing.post_shutdown_test()
class TestLocalizationModeShutdown(unittest.TestCase):
    def test_exits_cleanly_through_on_shutdown(self, proc_info, proc_output, launch_process):
        launch_testing.asserts.assertExitCodes(proc_info)
        launch_testing.asserts.assertInStream(
            proc_output, "on_shutdown in localization mode", launch_process, strip_ansi_escape_sequences=True,
            stream=OUTPUT_STREAM,
        )
        # A SIGTERM-style kill of the children would print this (Issue #43 section 2, review H2).
        self.assertNotIn("orphaned processes", read_all_output_text(proc_output))
