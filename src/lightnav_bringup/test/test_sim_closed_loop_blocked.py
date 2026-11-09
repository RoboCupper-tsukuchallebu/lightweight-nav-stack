import pytest
from closed_loop_helpers import ClosedLoopTest, description
from std_srvs.srv import Trigger

from lightnav_msgs.msg import FollowStatus


@pytest.mark.launch_test
def generate_test_description():
    return description("blocked", "base_link")


class TestClosedLoop(ClosedLoopTest):
    def test_blocked_and_resume(self):
        result = self.start_mission()
        self.wait_for(lambda: any(abs(message.linear.x) > 0.05 for _, message in self.commands))
        self.assertTrue(any(not message.is_stop_zone_occupied for message in self.obstacles))
        # The stop zone must trip or the controller must enter BLOCKED.
        self.wait_for(lambda: any(item.is_stop_zone_occupied for item in self.obstacles)
                      or any(item.state == FollowStatus.BLOCKED for item in self.status))
        self.wait_for(lambda: any(item.is_stuck for item in self.feedback))
        self.wait_for(lambda: any(item.is_stuck for item in self.status))
        if any(item.is_stop_zone_occupied for item in self.obstacles):
            stop_time = next(timestamp for timestamp, message in self.obstacle_receipts
                             if message.is_stop_zone_occupied)
            self.wait_for(lambda: any(timestamp >= stop_time and abs(message.linear.x) < 1e-6
                                      and abs(message.angular.z) < 1e-6 for timestamp, message in self.commands))
            zero_time = next(timestamp for timestamp, message in self.commands
                             if timestamp >= stop_time and abs(message.linear.x) < 1e-6
                             and abs(message.angular.z) < 1e-6)
            self.assertLessEqual(zero_time - stop_time, 0.5)
        blocked = next(item for item in self.status if item.state == FollowStatus.BLOCKED)
        stuck = next(item for item in self.status if item.is_stuck)
        def stamp_seconds(item):
            return item.header.stamp.sec + item.header.stamp.nanosec * 1e-9
        # Controller status is sampled at 20 Hz, so allow one tick of sampling error.
        self.assertGreaterEqual(stamp_seconds(stuck) - stamp_seconds(blocked), 9.9)
        client = self.node.create_client(Trigger, "/lightnav/sim/clear_obstacles")
        self.assertTrue(client.wait_for_service(timeout_sec=60.0))
        future = client.call_async(Trigger.Request())
        self.wait_for(future.done, 60.0)
        self.assertTrue(future.result().success)
        self.assert_success(result)
