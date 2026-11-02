"""pytest plugin loaded by `pixi run test` (PYTEST_PLUGINS).

1. Refuses to run on ROS domain 42 (real robot) or 43 (GUI / default domain).
2. Restores DYLD_LIBRARY_PATH on macOS. colcon starts tests through /bin/sh and
   /usr/bin/env, where SIP strips every DYLD_* variable, so the task passes the
   value under LIGHTNAV_DYLD_LIBRARY_PATH and it is put back here. rcpputils
   reads DYLD_LIBRARY_PATH at call time to locate typesupport libraries.
"""
import os
import sys

import pytest

FORBIDDEN_ROS_DOMAIN_IDS = ("42", "43")
SIP_SAFE_VARIABLE = "LIGHTNAV_DYLD_LIBRARY_PATH"


def pytest_configure(config):
    domain_id = os.environ.get("ROS_DOMAIN_ID")
    if domain_id is None or domain_id in FORBIDDEN_ROS_DOMAIN_IDS:
        raise pytest.UsageError(
            f"ROS_DOMAIN_ID={domain_id!r} is not allowed for tests. "
            "Run through `pixi run test` (domain 44, LOCALHOST)."
        )
    library_path = os.environ.get(SIP_SAFE_VARIABLE)
    if sys.platform == "darwin" and library_path:
        existing = os.environ.get("DYLD_LIBRARY_PATH")
        os.environ["DYLD_LIBRARY_PATH"] = (
            f"{library_path}:{existing}" if existing else library_path
        )
