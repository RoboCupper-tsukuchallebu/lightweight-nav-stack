import os
import subprocess
import sys
from pathlib import Path

PLUGIN_DIRECTORY = Path(__file__).resolve().parents[3] / "tools" / "pytest_plugins"
USAGE_ERROR_EXIT_CODE = 4


def run_trivial_pytest(tmp_path, domain_id):
    (tmp_path / "test_trivial.py").write_text("def test_trivial():\n    pass\n")
    environment = {
        **os.environ,
        "ROS_DOMAIN_ID": domain_id,
        "PYTEST_PLUGINS": "lightnav_test_environment",
        "PYTHONPATH": str(PLUGIN_DIRECTORY),
    }
    environment.pop("LIGHTNAV_DYLD_LIBRARY_PATH", None)
    return subprocess.run(
        [sys.executable, "-m", "pytest", "-p", "no:cacheprovider", str(tmp_path)],
        check=False,
        env=environment,
        cwd=tmp_path,
        capture_output=True,
        text=True,
    )


def test_plugin_aborts_pytest_on_wrong_domain(tmp_path):
    result = run_trivial_pytest(tmp_path, "99")
    assert result.returncode == USAGE_ERROR_EXIT_CODE, result.stdout + result.stderr


def test_plugin_lets_pytest_run_on_domain_44(tmp_path):
    result = run_trivial_pytest(tmp_path, "44")
    assert result.returncode == 0, result.stdout + result.stderr
