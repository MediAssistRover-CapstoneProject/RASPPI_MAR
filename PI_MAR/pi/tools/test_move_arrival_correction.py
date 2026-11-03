"""Recorded post-Spin XY corrections must not terminate an otherwise completed route."""
import sys,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch,Mock,AsyncMock
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'pi/tools')]
import test_move_position_only as fixtures
from mediassist.contracts import StepError

class ArrivalCorrectionTests(unittest.IsolatedAsyncioTestCase):
    def setup_adapter(self):
        self.fixture=fixtures.ArrivalTests();a=self.fixture.adapter()
        a.worker['navigation_runtime']['arrival_mode']='POSITION_THEN_SPIN'
        return a

    def test_recorded_xy_corrections_preserve_actual_pose_after_a_completed_turn(self):
        for error,yaw in ((.10899452248142039,.0862909496858868),(.10246155963140482,.027650847621327083)):
            a=self.setup_adapter();self.fixture.pose.update(x_m=1.-error,y_m=0.,yaw_rad=yaw)
            actual,measured,angle=a._verify_endpoint_arrival('A',heading_completed=True)
            self.assertAlmostEqual(measured,error);self.assertEqual(actual,self.fixture.pose)
            self.assertAlmostEqual(angle,yaw)
            a.node.get_logger().warning.assert_not_called()

    def test_stale_pose_far_goal_and_uncompleted_turn_still_fail(self):
        a=self.setup_adapter();self.fixture.pose.update(x_m=.87,yaw_rad=0.)
        with self.assertRaisesRegex(StepError,'ENDPOINT_FINAL_POSE_NOT_CONFIRMED'):
            a._verify_endpoint_arrival('A',heading_completed=True)
        a.node.get_logger().info.assert_not_called()
        a.telemetry=lambda:{'map_pose':{'fresh':False}}
        with self.assertRaisesRegex(StepError,'LOCALIZATION_LOST_AT_GOAL'):
            a._verify_endpoint_arrival('A',heading_completed=True)
        a=self.setup_adapter();self.fixture.pose.update(x_m=.96,yaw_rad=.3)
        with self.assertRaisesRegex(StepError,'ENDPOINT_FINAL_POSE_NOT_CONFIRMED'):a._verify_endpoint_arrival('A')

    async def test_two_move_steps_continue_after_recorded_post_turn_correction_without_extra_goals(self):
        a=self.setup_adapter();a.cfg['endpoints']['B']=dict(x_m=2.,y_m=0.,yaw_rad=0.)
        a.worker['navigation_session']={'session_id':'session'}
        a._relay_navigation_plans=AsyncMock(side_effect=AssertionError('No realtime path upload during MOVE'))
        async def align(p,eid):self.fixture.pose.update(x_m=a.cfg['endpoints'][eid]['x_m']-.109,yaw_rad=.086)
        a._finish_heading.side_effect=align
        with patch.dict(sys.modules,{'nav2_msgs.action':N(NavigateToPose=N(Goal=lambda:N()))}):
            first=await a._move_one({'max_speed_mps':.4},'A')
            second=await a._move_one({'max_speed_mps':.4},'B')
        self.assertTrue(first['reached']);self.assertTrue(second['reached'])
        self.assertEqual(a.nav.send_goal_async.call_count,2)
        self.assertEqual(a._finish_heading.await_count,2)
        a._relay_navigation_plans.assert_not_awaited()
        self.assertAlmostEqual(first['position_error_m'],.109)
        self.assertAlmostEqual(second['position_error_m'],.109)

    async def test_acceptable_correction_does_not_wait_or_send_an_extra_motion(self):
        a=self.setup_adapter();self.fixture.pose.update(x_m=.891,yaw_rad=.086)
        with patch('mediassist.ros_adapter.asyncio.sleep',new=AsyncMock()) as sleep:
            await a._settle_final_pose('A');sleep.assert_not_awaited()
        a.nav.send_goal_async.assert_not_called();a._finish_heading.assert_not_awaited()

if __name__=='__main__':unittest.main()
