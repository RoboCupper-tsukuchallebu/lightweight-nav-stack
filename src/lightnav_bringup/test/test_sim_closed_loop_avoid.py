import pytest
from closed_loop_helpers import ClosedLoopTest, description


@pytest.mark.launch_test
def generate_test_description():
    return description("avoid", "base_link")


class TestClosedLoop(ClosedLoopTest):
    def test_avoidance(self):
        self.run_avoidance()
