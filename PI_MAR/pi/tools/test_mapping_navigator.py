"""Exercise Nav2 action lifecycle and handoff without ROS or motor publishers."""
import asyncio,sys,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import AsyncMock,Mock,patch
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'pi'))
from mediassist.mapping_navigator import MappingNavigator
from mediassist.contracts import StepError

def pose_message():return N(header=N(frame_id='',stamp=None),pose=N(position=N(x=0.,y=0.),orientation=N(z=0.,w=1.)))

class NavigatorTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.result=asyncio.get_running_loop().create_future();self.handle=N(accepted=True,get_result_async=lambda:self.result)
        self.adapter=N(cfg={'ros':{'map_frame':'map'}},node=Mock(),nav=Mock(),goal=None,
            clear_navigation_velocity=Mock(),navigation_velocity=Mock(return_value=((.12,.04,.1),True)),
            future=AsyncMock(return_value=self.handle),stop=AsyncMock(return_value=True),_late_cancel=Mock())
        self.adapter.nav.server_is_ready.return_value=True;self.exploration=N(exclude_goal=Mock())
        self.navigator=MappingNavigator(self.adapter,self.exploration)
        self.plan={'goal':dict(x_m=1.,y_m=0.,yaw_rad=.3)};self.pose=dict(x_m=0.,y_m=0.,yaw_rad=0.)
        self.modules={'geometry_msgs.msg':N(PoseStamped=pose_message),'nav2_msgs.action':N(NavigateToPose=N(Goal=lambda:N()))}
    async def test_goal_uses_map_frame_yaw_and_only_selected_fresh_nav2_velocity(self):
        with patch.dict(sys.modules,self.modules):velocity,reason=await self.navigator.tick(self.plan,self.pose,10.)
        self.assertEqual(velocity,(.12,.04,.1));self.assertEqual(reason,'FOLLOW_FRONTIER_NAV2')
        request=self.adapter.nav.send_goal_async.call_args.args[0]
        self.assertEqual(request.pose.header.frame_id,'map');self.assertAlmostEqual(request.pose.pose.orientation.z,__import__('math').sin(.15))
        self.adapter.clear_navigation_velocity.assert_called_once();self.assertIs(self.adapter.goal,self.handle)
        self.result.set_result(N(status=4))
        with patch.dict(sys.modules,self.modules):velocity,reason=await self.navigator.tick(self.plan,self.pose,11.)
        self.assertEqual(velocity,(0.,0.,0.));self.assertEqual(reason,'OBSERVING_FRONTIER');self.assertFalse(self.navigator.active)
        self.adapter.stop.assert_awaited_once();self.exploration.exclude_goal.assert_called_once()
    async def test_rejected_goal_is_not_replaced_by_reactive_driving(self):
        self.handle.accepted=False
        with patch.dict(sys.modules,self.modules):velocity,reason=await self.navigator.tick(self.plan,self.pose,10.)
        self.assertEqual(velocity,(0.,0.,0.));self.assertEqual(reason,'REPLANNING_REJECTED_FRONTIER');self.assertFalse(self.navigator.active)
        self.adapter.navigation_velocity.assert_not_called()
    async def test_stale_velocity_fails_and_stop_confirms_goal_cancellation(self):
        with patch.dict(sys.modules,self.modules):await self.navigator.tick(self.plan,self.pose,10.)
        self.adapter.navigation_velocity.return_value=((1.,1.,1.),False)
        with patch.dict(sys.modules,self.modules),self.assertRaisesRegex(StepError,'NAV2_COMMAND_STALE'):
            await self.navigator.tick(self.plan,self.pose,12.1)
        await self.navigator.stop();self.adapter.stop.assert_awaited_once();self.assertFalse(self.navigator.active)
    async def test_missing_server_stays_stopped(self):
        self.adapter.nav.server_is_ready.return_value=False
        with patch.dict(sys.modules,self.modules),self.assertRaisesRegex(StepError,'NAV2_MAPPING_NOT_READY'):
            await self.navigator.tick(self.plan,self.pose,10.)
        self.adapter.nav.send_goal_async.assert_not_called();self.adapter.navigation_velocity.assert_not_called()

if __name__=='__main__':unittest.main()
