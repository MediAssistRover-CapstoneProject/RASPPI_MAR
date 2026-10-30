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
        self.assertEqual(self.exploration.exclude_goal.call_args.kwargs['radius'],.12)
    async def test_near_viewpoint_does_not_blacklist_small_initial_map(self):
        from mediassist.frontier_mapping import plan
        w=40;raw=bytearray([255]*(w*w))
        for y in range(8,32):
            for x in range(8,32):raw[y*w+x]=0
        snapshot=dict(width=w,height=w,resolution=.025,origin=[-.5,-.5,0.],frame_id='map',data=bytes(raw))
        cfg=dict(frontier_min_length_m=.15,footprint=dict(front=.12,rear=.12,left=.1,right=.1),
                 stop_margin_m=.05,frontier_controller='nav2')
        pose=dict(x_m=.1,y_m=.0125,yaw_rad=0.)
        first=plan(snapshot,pose,cfg)
        with patch.dict(sys.modules,self.modules):
            velocity,reason=await self.navigator.tick(first,pose,10.)
        self.assertEqual(velocity,(0.,0.,0.));self.assertEqual(reason,'OBSERVING_FRONTIER')
        self.adapter.nav.send_goal_async.assert_not_called()
        call=self.exploration.exclude_goal.call_args;goal=call.args[0]
        pair=(goal['x_m'],goal['y_m'])
        # The former 35 cm exclusion blocked all reachable viewpoints.
        self.assertEqual(plan(snapshot,pose,{**cfg,'_excluded_goals':[pair]})['state'],'BLOCKED')
        next_plan=plan(snapshot,pose,{**cfg,'_excluded_goals':[(*pair,call.kwargs['radius'])]})
        self.assertEqual(next_plan['state'],'FRONTIERS_PRESENT')
        with patch.dict(sys.modules,self.modules):
            _,reason=await self.navigator.tick(next_plan,pose,11.)
        self.assertEqual(reason,'FOLLOW_FRONTIER_NAV2')
        self.adapter.nav.send_goal_async.assert_called_once()
    async def test_rejected_goal_is_not_replaced_by_reactive_driving(self):
        self.handle.accepted=False
        with patch.dict(sys.modules,self.modules):velocity,reason=await self.navigator.tick(self.plan,self.pose,10.)
        self.assertEqual(velocity,(0.,0.,0.));self.assertEqual(reason,'REPLANNING_REJECTED_FRONTIER');self.assertFalse(self.navigator.active)
        self.adapter.navigation_velocity.assert_not_called()
    async def test_stale_velocity_holds_zero_then_cancels_before_selecting_another_goal(self):
        with patch.dict(sys.modules,self.modules):await self.navigator.tick(self.plan,self.pose,10.)
        self.adapter.navigation_velocity.return_value=((1.,1.,1.),False)
        with patch.dict(sys.modules,self.modules):
            velocity,reason=await self.navigator.tick(self.plan,self.pose,12.1)
            self.assertEqual(velocity,(0.,0.,0.));self.assertEqual(reason,'WAITING_FOR_NAV2_VELOCITY')
            velocity,reason=await self.navigator.tick(self.plan,self.pose,15.5)
        self.assertEqual(velocity,(0.,0.,0.));self.assertEqual(reason,'REPLANNING_STALE_NAV2')
        self.adapter.stop.assert_awaited_once();self.assertFalse(self.navigator.active)
        self.exploration.exclude_goal.assert_called_once()
    async def test_goal_without_progress_is_stopped_and_excluded_instead_of_spinning_forever(self):
        with patch.dict(sys.modules,self.modules):
            await self.navigator.tick(self.plan,self.pose,10.)
            velocity,reason=await self.navigator.tick(self.plan,self.pose,41.)
        self.assertEqual(velocity,(0.,0.,0.));self.assertEqual(reason,'REPLANNING_STALLED_FRONTIER')
        self.adapter.stop.assert_awaited_once();self.assertFalse(self.navigator.active)
    async def test_missing_server_stays_stopped(self):
        self.adapter.nav.server_is_ready.return_value=False
        with patch.dict(sys.modules,self.modules),self.assertRaisesRegex(StepError,'NAV2_MAPPING_NOT_READY'):
            await self.navigator.tick(self.plan,self.pose,10.)
        self.adapter.nav.send_goal_async.assert_not_called();self.adapter.navigation_velocity.assert_not_called()

    async def test_detour_away_from_goal_counts_as_progress_but_total_time_is_bounded(self):
        with patch.dict(sys.modules,self.modules):
            await self.navigator.tick(self.plan,self.pose,10.)
            for now,y in ((25.,.1),(40.,.2),(55.,.3),(70.,.4),(85.,.5)):
                _,reason=await self.navigator.tick(self.plan,{**self.pose,'y_m':y},now)
                self.assertEqual(reason,'FOLLOW_FRONTIER_NAV2')
            velocity,reason=await self.navigator.tick(self.plan,{**self.pose,'y_m':.6},101.)
        self.assertEqual(reason,'REPLANNING_STALLED_FRONTIER');self.assertEqual(velocity,(0.,0.,0.))

if __name__=='__main__':unittest.main()
