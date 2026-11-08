"""localization.launch.py mode:=localization reaches ACTIVE and SIGINT goes through on_shutdown."""

import pytest
from localization_launch_support import (
    SHUTDOWN_TIMEOUT_SECONDS,
    LocalizationLaunchCase,
    build_launch_description,
)

OUTPUT_TIMEOUT_SECONDS = 10


@pytest.mark.launch_test
def generate_test_description():
    return build_launch_description("localization")


class TestLocalizationModeLaunch(LocalizationLaunchCase):
    def test_activates_and_shuts_down_on_sigint(self, proc_info, proc_output, launch_process):
        self.assertTrue(self.wait_until_active(), "localization_node did not become active")
        proc_output.assertWaitFor(
            "localization node active in localization mode", process=launch_process, timeout=OUTPUT_TIMEOUT_SECONDS,
            stream="stdout"
        )

        self.send_sigint_to(launch_process)

        proc_info.assertWaitForShutdown(process=launch_process, timeout=SHUTDOWN_TIMEOUT_SECONDS)
        proc_output.assertWaitFor(
            "on_shutdown in localization mode", process=launch_process, timeout=OUTPUT_TIMEOUT_SECONDS,
            stream="stdout"
        )
        # A SIGTERM-style kill of the children would print this (Issue #43 section 2, review H2).
        self.assertNotIn("orphaned processes", "".join(o.text.decode(errors="replace") for o in proc_output))
