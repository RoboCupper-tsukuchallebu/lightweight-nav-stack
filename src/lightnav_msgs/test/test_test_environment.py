import os

import rclpy

from lightnav_msgs.srv import PlanPath


def test_runs_on_isolated_test_domain():
    assert os.environ["ROS_DOMAIN_ID"] == "44"
    assert os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "LOCALHOST"


def test_faulthandler_timeout_is_enabled(pytestconfig):
    # pytest 8 returns the faulthandler_timeout ini value as a string.
    assert float(pytestconfig.getini("faulthandler_timeout") or 0) > 0


def test_custom_service_typesupport_is_loadable():
    # Fails with "type_support is null" when the typesupport libraries
    # cannot be dlopen'ed (macOS SIP dropping DYLD_LIBRARY_PATH).
    rclpy.init()
    try:
        node = rclpy.create_node("typesupport_probe")
        node.create_service(PlanPath, "typesupport_probe", lambda request, response: response)
        node.destroy_node()
    finally:
        rclpy.shutdown()
