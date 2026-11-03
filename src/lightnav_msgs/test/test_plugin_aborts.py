import lightnav_test_environment as plugin
import pytest


@pytest.mark.parametrize("bad_domain_id", [None, "", "99", "0"])
def test_domain_check_aborts_on_anything_but_44(bad_domain_id):
    with pytest.raises(pytest.UsageError):
        plugin.check_domain(bad_domain_id)


def test_domain_check_accepts_44():
    plugin.check_domain("44")


def test_restart_adds_dyld_and_marker():
    environment = plugin.restart_with_dyld({}, "/lib")
    assert environment["DYLD_LIBRARY_PATH"] == "/lib"
    assert environment[plugin.RESTARTED_MARKER]


def test_no_restart_when_dyld_present():
    assert plugin.restart_with_dyld({"DYLD_LIBRARY_PATH": "/lib"}, "/lib") is None


def test_aborts_instead_of_restarting_twice():
    with pytest.raises(pytest.UsageError):
        plugin.restart_with_dyld({plugin.RESTARTED_MARKER: "1"}, "/lib")
