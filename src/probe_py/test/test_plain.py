def test_plain():
    assert 1 + 1 == 2


def test_custom_service_type_loads():
    import rclpy

    from lightnav_msgs.srv import SetInitialPose

    rclpy.init()
    node = rclpy.create_node("probe")
    node.create_service(SetInitialPose, "probe", lambda request, response: response)
    node.destroy_node()
    rclpy.shutdown()
