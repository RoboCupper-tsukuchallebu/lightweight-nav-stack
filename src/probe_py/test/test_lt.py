import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([launch.actions.ExecuteProcess(cmd=["sleep", "1"]), launch_testing.actions.ReadyToTest()])

class TestRun(unittest.TestCase):
    def test_ready(self):
        self.assertTrue(True)
