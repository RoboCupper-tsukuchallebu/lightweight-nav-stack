import os
from xml.etree import ElementTree

import rclpy

from lightnav_msgs.srv import PlanPath


LOCALHOST_ADDRESS = "127.0.0.1"


def local_tag_name(element):
    """Tag name without the XML namespace (the Fast DDS profile declares one)."""
    return element.tag.rsplit("}", 1)[-1]


def find_localhost_only_violations(profiles_file):
    """Return why the Fast DDS profile can reach beyond localhost (an empty list means it cannot).

    The production profile (docker/dds/fastdds_profiles.xml) uses SHM plus UDPv4 on 127.0.0.1 only and no multicast.
    """
    violations = []
    elements = list(ElementTree.parse(profiles_file).getroot().iter())

    use_builtin_transports = [e.text for e in elements if local_tag_name(e) == "useBuiltinTransports"]
    if use_builtin_transports != ["false"]:
        violations.append(f"useBuiltinTransports must be exactly one 'false', got {use_builtin_transports}")

    udp_descriptors = [
        descriptor
        for descriptor in elements
        if local_tag_name(descriptor) == "transport_descriptor"
        and any(local_tag_name(e) == "type" and e.text == "UDPv4" for e in descriptor)
    ]
    for descriptor in udp_descriptors:
        white_list = [
            address.text
            for white_list_element in descriptor
            if local_tag_name(white_list_element) == "interfaceWhiteList"
            for address in white_list_element
            if local_tag_name(address) == "address"
        ]
        if white_list != [LOCALHOST_ADDRESS]:
            violations.append(f"UDPv4 interfaceWhiteList must be only {LOCALHOST_ADDRESS}, got {white_list}")

    if any(local_tag_name(e).lower().endswith("multicastlocatorlist") for e in elements):
        violations.append("no multicast locator list may be set")
    return violations


def test_runs_on_isolated_test_domain():
    assert os.environ["ROS_DOMAIN_ID"] == "44"
    discovery_range = os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"]
    if discovery_range == "LOCALHOST":  # pixi
        return
    # docker/Dockerfile `test` stage: the production Fast DDS profile with SYSTEM_DEFAULT, because LOCALHOST
    # would add a default SHM transport after the XML. The profile must keep the traffic on localhost.
    assert discovery_range == "SYSTEM_DEFAULT"
    profiles_file = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE")
    assert profiles_file, "SYSTEM_DEFAULT needs FASTRTPS_DEFAULT_PROFILES_FILE"
    assert find_localhost_only_violations(profiles_file) == []


def test_profile_check_rejects_a_profile_that_reaches_the_lan(tmp_path):
    broken_profile = tmp_path / "broken.xml"
    broken_profile.write_text(
        '<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">'
        "<transport_descriptors><transport_descriptor><transport_id>udp</transport_id><type>UDPv4</type>"
        "<interfaceWhiteList><address>192.168.0.1</address></interfaceWhiteList>"
        "</transport_descriptor></transport_descriptors>"
        '<participant profile_name="p" is_default_profile="true"><rtps>'
        "<useBuiltinTransports>true</useBuiltinTransports>"
        "<builtin><metatrafficMulticastLocatorList/><multicastLocatorList/></builtin>"
        "</rtps></participant></profiles>"
    )
    assert len(find_localhost_only_violations(broken_profile)) == 3


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
