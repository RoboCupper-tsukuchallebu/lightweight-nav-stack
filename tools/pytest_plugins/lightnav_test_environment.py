"""pytest plugin loaded by `pixi run test` (PYTEST_PLUGINS).

1. Aborts unless ROS_DOMAIN_ID is exactly "44" (never the robot's 42 or the GUI's 43).
2. Restores DYLD_LIBRARY_PATH on macOS. colcon starts tests through /bin/sh and
   /usr/bin/env, where SIP strips every DYLD_* variable, so the task passes the
   value under LIGHTNAV_DYLD_LIBRARY_PATH and pytest is restarted with it here.
"""
import os
import sys

import pytest

TEST_ROS_DOMAIN_ID = "44"
SIP_SAFE_VARIABLE = "LIGHTNAV_DYLD_LIBRARY_PATH"
RESTARTED_MARKER = "LIGHTNAV_PYTEST_RESTARTED"


def check_domain(domain_id):
    if domain_id != TEST_ROS_DOMAIN_ID:
        raise pytest.UsageError(
            f"ROS_DOMAIN_ID={domain_id!r} is not allowed for tests (must be "
            f"{TEST_ROS_DOMAIN_ID!r}). Run through `pixi run test`."
        )


def restart_with_dyld(environment, library_path):
    """Return the environment to restart with, or None if no restart is needed."""
    if library_path in environment.get("DYLD_LIBRARY_PATH", ""):
        return None
    if environment.get(RESTARTED_MARKER):
        raise pytest.UsageError(
            "DYLD_LIBRARY_PATH is still missing after restarting pytest; "
            "refusing to restart again."
        )
    return {**environment, "DYLD_LIBRARY_PATH": library_path, RESTARTED_MARKER: "1"}


def pytest_configure(config):
    check_domain(os.environ.get("ROS_DOMAIN_ID"))
    library_path = os.environ.get(SIP_SAFE_VARIABLE)
    if sys.platform != "darwin" or not library_path:
        return
    restarted_environment = restart_with_dyld(os.environ, library_path)
    if restarted_environment is not None:
        # dyld reads DYLD_LIBRARY_PATH only at process start.
        os.execve(sys.executable, sys.orig_argv, restarted_environment)
