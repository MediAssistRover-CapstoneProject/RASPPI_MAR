"""Exercise Nav2 action lifecycle and handoff without ROS or motor publishers."""
import asyncio,sys,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import AsyncMock,Mock,patch
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'pi'))
from mediassist.mapping_navigator import MappingNavigator,translation_sweep_clear,rotation_sweep_clear
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
        self.assertEqual(velocity,(0.,0.,0.));self.assertEqual(reason,'REPLANNING_REJECTED_FRONTIER')
        self.assertIsNone(self.navigator.target);self.assertIsNone(self.navigator.side)
        self.assertTrue(self.navigator.side_pending) # request a validated escape, never drive from rejection alone
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
        self.adapter.stop.assert_awaited_once();self.assertIsNone(self.navigator.target)
        self.assertIsNone(self.navigator.side);self.assertTrue(self.navigator.side_pending)
    async def test_missing_server_stays_stopped(self):
        self.adapter.nav.server_is_ready.return_value=False
        with patch.dict(sys.modules,self.modules),self.assertRaisesRegex(StepError,'NAV2_MAPPING_NOT_READY'):
            await self.navigator.tick(self.plan,self.pose,10.)
        self.adapter.nav.send_goal_async.assert_not_called();self.adapter.navigation_velocity.assert_not_called()

    async def test_detour_away_from_goal_counts_as_progress_but_total_time_is_bounded(self):
        self.exploration.cfg={'frontier_goal_timeout_sec':90.,'frontier_stall_sec':20.}
        with patch.dict(sys.modules,self.modules):
            await self.navigator.tick(self.plan,self.pose,10.)
            for now,y in ((25.,.1),(40.,.2),(55.,.3),(70.,.4),(85.,.5)):
                _,reason=await self.navigator.tick(self.plan,{**self.pose,'y_m':y},now)
                self.assertEqual(reason,'FOLLOW_FRONTIER_NAV2')
            velocity,reason=await self.navigator.tick(self.plan,{**self.pose,'y_m':.6},101.)
        self.assertEqual(reason,'REPLANNING_STALLED_FRONTIER');self.assertEqual(velocity,(0.,0.,0.))

    async def test_braking_veto_replans_before_stall_without_counting_stationary_turn(self):
        with patch.dict(sys.modules,self.modules):
            await self.navigator.tick(self.plan,self.pose,10.)
            self.navigator.path=((0.,0.),(.3,0.))
            self.navigator.note_velocity_limit((0.,0.,.4),(0.,0.,0.),10.1)
            self.assertIsNone(self.navigator.velocity_blocked_at)
            self.navigator.note_velocity_limit((.1,0.,.2),(0.,0.,0.),10.2)
            self.navigator.note_velocity_limit((.1,0.,.2),(0.,0.,0.),10.6)
            _,reason=await self.navigator.tick(self.plan,self.pose,10.81)
            self.assertEqual(reason,'FOLLOW_FRONTIER_NAV2')
            self.adapter.stop.assert_not_awaited()
            velocity,reason=await self.navigator.tick(self.plan,self.pose,11.71)
        self.assertEqual(reason,'REPLANNING_BLOCKED_FRONTIER');self.assertEqual(velocity,(0.,0.,0.))
        self.adapter.stop.assert_awaited_once();self.assertTrue(self.navigator.side_pending)
        self.navigator.note_velocity_limit((.1,0.,0.),(.03,0.,0.),10.9)
        self.assertIsNone(self.navigator.velocity_blocked_at)

    def test_fixed_yaw_recovery_can_choose_reverse_or_diagonal_and_keeps_bounds(self):
        snapshot=dict(width=80,height=80,resolution=.025,origin=[-1.,-1.,0.],data=bytes(80*80))
        cfg=dict(vehicle='rigid',footprint=dict(front=.1,rear=.1,left=.08,right=.08),
                 stop_margin_m=.02,speed_mps=.18,reverse_enabled=True,max_reverse_mps=.05,lidar_stop_distance_m=.05,
                 frontier_observation_rotation=False)
        self.exploration.cfg=cfg
        self.adapter.cfg['motion']=dict(chassis_model='MENTORPI_M1_MECANUM',allow_strafe=True,
                                        allow_reverse=True,max_reverse_speed_mps=.04)
        self.adapter.odom_pose_sample=((0.,0.,0.),10.)
        self.adapter.odom_twist_sample=((0.,0.,0.),10.)
        self.navigator.path=((0.,0.),(.3,.3));odom=dict(value=0.,age_sec=.01)
        clear=dict(valid=True,nearest_lidar_m=1.,obstacle_points=[])
        self.navigator.start_sideways('run',self.pose,10.,snapshot,clear,odom)
        self.assertIsNotNone(self.navigator.side)
        translation=self.navigator.side[-1]
        self.assertGreater(translation[0],0.);self.assertAlmostEqual(translation[0],translation[1])
        self.assertAlmostEqual(__import__('math').hypot(*translation),.15)
        velocity=self.navigator.sideways_velocity(self.pose,10.1,snapshot,clear)
        self.assertAlmostEqual(velocity[0],velocity[1]);self.assertEqual(velocity[2],0.)
        self.assertEqual(self.navigator.sideways_velocity(self.pose,11.,snapshot,clear),(0.,0.,0.))
        self.navigator.side=None;self.navigator.side_attempts=[]
        blocked={**clear,'nearest_lidar_m':.12,'obstacle_points':[(.15,0.),(0.,.12),(0.,-.12)]}
        self.navigator.path=((0.,0.),(.3,0.))
        self.navigator.start_sideways('run',self.pose,10.,snapshot,blocked,odom)
        self.assertIsNotNone(self.navigator.side)
        self.assertLess(self.navigator.side[-1][0],0.);self.assertEqual(self.navigator.side[-1][1],0.)
        velocity=self.navigator.sideways_velocity(self.pose,10.1,snapshot,blocked)
        self.assertGreaterEqual(velocity[0],-.04)
        self.assertLess(velocity[0],0.);self.assertEqual(velocity[1:],(0.,0.))
        # A valid 15 cm maneuver may last beyond the old four-second limit.
        self.adapter.odom_pose_sample=((- .09,0.,0.),14.1)
        progressed={**self.pose,'x_m':-.09}
        self.assertLess(self.navigator.sideways_velocity(progressed,14.1,snapshot,blocked)[0],0.)
        # A longer time budget must not keep a motionless robot waiting.
        self.adapter.odom_pose_sample=((- .09,0.,0.),15.7)
        self.assertEqual(self.navigator.sideways_velocity(progressed,15.7,snapshot,blocked),(0.,0.,0.))

    def test_recovery_sweep_refuses_unseen_map_cells_scan_hits_and_keepout(self):
        snapshot=dict(width=80,height=80,resolution=.025,origin=[-1.,-1.,0.],data=bytes(80*80))
        body=dict(front=.1,rear=.1,left=.08,right=.08)
        clear=dict(valid=True,nearest_lidar_m=1.,obstacle_points=[])
        self.assertTrue(translation_sweep_clear(snapshot,self.pose,clear,body,.02,(.15,0.)))
        self.assertFalse(translation_sweep_clear(snapshot,self.pose,clear,body,.02,(.151,0.)))

    def test_recovery_waits_for_residual_motion_then_tries_without_a_path(self):
        snapshot,view=self.observation_scene()
        self.exploration.cfg['frontier_observation_rotation']=False
        self.navigator.side_pending=True
        self.adapter.odom_twist_sample=((.03,0.,0.),10.)
        self.navigator.start_sideways('wait-stop',self.pose,10.,snapshot,view,dict(value=.03,age_sec=.01))
        self.assertTrue(self.navigator.side_pending)
        self.assertEqual(self.navigator.recovery_reason,'WAITING_FOR_RECOVERY_STOP')
        self.assertIsNone(self.navigator.side)
        self.adapter.odom_pose_sample=((0.,0.,0.),10.2)
        self.adapter.odom_twist_sample=((0.,0.,0.),10.2)
        self.navigator.start_sideways('wait-stop',self.pose,10.2,snapshot,view,dict(value=0.,age_sec=.01))
        self.assertFalse(self.navigator.side_pending);self.assertIsNotNone(self.navigator.side)
        self.navigator=MappingNavigator(self.adapter,self.exploration)
        self.adapter.odom_pose_sample=None
        self.navigator.start_sideways('timeout',self.pose,11.,snapshot,view,dict(value=0.,age_sec=.01))
        self.assertTrue(self.navigator.side_pending)
        self.navigator.start_sideways('timeout',self.pose,13.01,snapshot,view,dict(value=0.,age_sec=.01))
        self.assertFalse(self.navigator.side_pending)
        self.assertEqual(self.navigator.recovery_reason,'RECOVERY_STOP_TIMEOUT')

    def test_no_path_recovery_leaves_padding_but_not_physical_contact(self):
        snapshot,clear=self.observation_scene()
        cfg=self.exploration.cfg;cfg.update(stop_margin_m=.01,frontier_observation_rotation=False)
        body=cfg['footprint'];obstacle=(body['front']+.005,0.)
        view={**clear,'nearest_lidar_m':obstacle[0],'obstacle_points':[obstacle]}
        self.navigator.start_sideways('no-path',self.pose,10.,snapshot,view,dict(value=0.,age_sec=.01))
        self.assertIsNotNone(self.navigator.side)
        self.assertLess(self.navigator.side[-1][0],0.)
        self.assertEqual(self.navigator.side[-1][1],0.)
        self.assertTrue(translation_sweep_clear(snapshot,self.pose,view,body,.01,(-.10,0.)))
        self.assertFalse(translation_sweep_clear(snapshot,self.pose,view,body,.01,(.10,0.)))
        contact={**view,'obstacle_points':[(body['front']-.001,0.)]}
        self.assertFalse(translation_sweep_clear(snapshot,self.pose,contact,body,.01,(-.10,0.)))
        zones=N(validate_path=Mock(side_effect=StepError('NAVIGATION_KEEPOUT')))
        self.assertFalse(translation_sweep_clear(snapshot,self.pose,view,body,.01,(-.10,0.),zones))
        # Escaping one padded contact must not skim a new wall midway through
        # the segment, including profiles with a larger configured reserve.
        wide_body=dict(front=.1,rear=.1,left=.08,right=.08)
        crossing={**clear,'obstacle_points':[(-.105,0.),(.175,-.03)]}
        fine_grid=dict(width=100,height=100,resolution=.01,origin=[-.5,-.5,0.],data=bytes(10000))
        self.assertFalse(translation_sweep_clear(fine_grid,self.pose,crossing,wide_body,.04,(.106,.106)))
        raw=bytearray(snapshot['data']);raw[40*80+44]=255
        self.assertFalse(translation_sweep_clear({**snapshot,'data':bytes(raw)},self.pose,view,body,.01,(-.10,0.)))
        farther={**clear,'obstacle_points':[(.25,0.)]}
        self.assertTrue(translation_sweep_clear(snapshot,self.pose,farther,body,.02,(.10,0.)))
        self.assertFalse(translation_sweep_clear(snapshot,self.pose,farther,body,.02,(.15,0.)))
        self.assertTrue(translation_sweep_clear(snapshot,self.pose,clear,body,.02,(.07,.07)))
        self.assertFalse(translation_sweep_clear(snapshot,self.pose,{**clear,'obstacle_points':[(.16,.1)]},body,.02,(.07,.07)))
        raw=bytearray(snapshot['data']);raw[45*80+46]=255
        self.assertFalse(translation_sweep_clear({**snapshot,'data':bytes(raw)},self.pose,clear,body,.02,(.07,.07)))
        zones=N(validate_path=Mock(side_effect=StepError('NAVIGATION_KEEPOUT')))
        self.assertFalse(translation_sweep_clear(snapshot,self.pose,clear,body,.02,(.07,.07),zones))

    def observation_scene(self):
        snapshot=dict(width=80,height=80,resolution=.025,origin=[-1.,-1.,0.],data=bytes(80*80))
        self.exploration.cfg=dict(vehicle='rigid',footprint=dict(front=.1,rear=.1,left=.08,right=.08),
            stop_margin_m=.02,speed_mps=.25,angular_rps=.8,reverse_enabled=True,max_reverse_mps=.15,
            lidar_stop_distance_m=.05,scan_max_age_sec=.4,frontier_observation_rotation=True)
        self.adapter.cfg['motion']=dict(chassis_model='MENTORPI_M1_MECANUM',allow_strafe=True,
                                       allow_reverse=True,max_reverse_speed_mps=.15)
        self.adapter.odom_pose_sample=((0.,0.,0.),10.)
        self.adapter.odom_twist_sample=((0.,0.,0.),10.)
        return snapshot,dict(valid=True,nearest_lidar_m=1.,obstacle_points=[])

    def test_observation_without_path_counts_a_complete_turn_across_yaw_wrap(self):
        import math
        snapshot,view=self.observation_scene()
        self.navigator.start_translation=Mock(return_value=False)
        self.navigator.start_sideways('observation',self.pose,10.,snapshot,view,dict(value=0.,age_sec=.01))
        self.assertIsNotNone(self.navigator.observation);self.assertIsNone(self.navigator.side)
        self.assertAlmostEqual(self.navigator.observation['target'],math.tau)
        for index in range(1,64):
            angle=index*.1;now=10.+index*.125;yaw=math.atan2(math.sin(angle),math.cos(angle))
            self.adapter.odom_pose_sample=((0.,0.,yaw),now)
            self.adapter.odom_twist_sample=((0.,0.,.8),now)
            velocity=self.navigator.sideways_velocity({**self.pose,'yaw_rad':yaw},now,snapshot,view)
            if index<63:self.assertGreater(velocity[2],0.)
        self.assertEqual(velocity,(0.,0.,0.));self.assertIsNone(self.navigator.observation)
        self.adapter.odom_pose_sample=((0.,0.,0.),18.)
        self.adapter.odom_twist_sample=((0.,0.,0.),18.)
        self.navigator.start_sideways('observation',self.pose,18.,snapshot,view,dict(value=0.,age_sec=.01))
        self.assertIsNone(self.navigator.observation,'Do not repeat a full turn at the same location')

    def test_observation_stops_for_new_obstacle_stale_odom_or_center_drift(self):
        for reason in ('obstacle','odom','map_drift','odom_drift','angle_disagreement','no_progress'):
            snapshot,view=self.observation_scene()
            self.navigator.observation=None;self.navigator.observation_attempts=[]
            self.navigator.start_translation=Mock(return_value=False)
            self.navigator.start_sideways(reason,self.pose,10.,snapshot,view,dict(value=0.,age_sec=.01))
            self.assertIsNotNone(self.navigator.observation)
            now=11.6 if reason=='no_progress' else 10.1;pose=dict(self.pose)
            self.adapter.odom_pose_sample=((0.,0.,0.),now)
            self.adapter.odom_twist_sample=((0.,0.,0.),now)
            if reason=='obstacle':view={**view,'obstacle_points':[(.1,0.)]}
            if reason=='odom':self.adapter.odom_pose_sample=((0.,0.,0.),9.)
            if reason=='map_drift':pose['x_m']=.05
            if reason=='odom_drift':self.adapter.odom_pose_sample=((.03,0.,0.),now)
            if reason=='angle_disagreement':pose['yaw_rad']=.2
            self.assertEqual(self.navigator.sideways_velocity(pose,now,snapshot,view),(0.,0.,0.),reason)
            self.assertIsNone(self.navigator.observation)

    def test_blocked_full_rotation_can_translate_away_instead(self):
        import math
        snapshot,view=self.observation_scene()
        body=self.exploration.cfg['footprint']
        self.assertTrue(rotation_sweep_clear(snapshot,self.pose,view,body,.02,math.tau))
        unknown=bytearray(snapshot['data']);unknown[43*80+45]=255
        self.assertFalse(rotation_sweep_clear({**snapshot,'data':bytes(unknown)},self.pose,view,body,.02,math.tau))
        zones=N(validate_path=Mock(side_effect=StepError('NAVIGATION_KEEPOUT')))
        self.assertFalse(rotation_sweep_clear(snapshot,self.pose,view,body,.02,math.tau,zones))
        view={**view,'obstacle_points':[(.145,0.)],'nearest_lidar_m':.145}
        self.assertFalse(rotation_sweep_clear(snapshot,self.pose,view,self.exploration.cfg['footprint'],.02,math.tau))
        self.navigator.path=((0.,0.),(-.3,0.))
        self.navigator.start_sideways('blocked-observation',self.pose,10.,snapshot,view,dict(value=0.,age_sec=.01))
        self.assertIsNone(self.navigator.observation);self.assertIsNotNone(self.navigator.side)
        self.assertLess(self.navigator.side[-1][0],0.)
        # After a 10 cm retreat, rotation can be used if no useful translation remains.
        self.navigator.side=None;shifted={**self.pose,'x_m':-.1}
        self.adapter.odom_pose_sample=((- .1,0.,0.),11.)
        self.adapter.odom_twist_sample=((0.,0.,0.),11.)
        clear={**view,'obstacle_points':[(.245,0.)],'nearest_lidar_m':.245}
        self.navigator.start_translation=Mock(return_value=False)
        self.navigator.start_sideways('blocked-observation',shifted,11.,snapshot,clear,dict(value=0.,age_sec=.01))
        self.assertIsNotNone(self.navigator.observation)

    async def test_stop_confirms_manual_observation_is_stopped(self):
        snapshot,view=self.observation_scene()
        self.navigator.start_translation=Mock(return_value=False)
        self.navigator.start_sideways('stop-observation',self.pose,10.,snapshot,view,dict(value=0.,age_sec=.01))
        self.assertIsNotNone(self.navigator.observation)
        await self.navigator.stop()
        self.adapter.stop.assert_awaited_once();self.assertIsNone(self.navigator.observation)

    def test_recovery_respects_one_centimetre_config_without_an_extra_two_centimetre_clamp(self):
        for margin in (.01,.02):
            snapshot,clear=self.observation_scene()
            self.exploration.cfg.update(stop_margin_m=margin,frontier_observation_rotation=False)
            self.navigator=MappingNavigator(self.adapter,self.exploration)
            self.navigator.path=((0.,0.),(.3,0.))
            side_wall={**clear,'nearest_lidar_m':.099,'obstacle_points':[(0.,.099)]}
            self.assertEqual(translation_sweep_clear(snapshot,self.pose,side_wall,
                self.exploration.cfg['footprint'],margin,(.05,0.)),margin==.01)
            self.navigator.start_sideways('margin',self.pose,10.,snapshot,side_wall,dict(value=0.,age_sec=.01))
            self.assertIsNotNone(self.navigator.side)
            velocity=self.navigator.sideways_velocity(self.pose,10.1,snapshot,side_wall)
            self.assertGreater(__import__('math').hypot(*velocity[:2]),0.)
            self.assertLessEqual(velocity[1],0.)
            if margin==.02:self.assertLess(velocity[1],0.)
            self.navigator=MappingNavigator(self.adapter,self.exploration)
            self.exploration.cfg['frontier_observation_rotation']=True
            corner={**clear,'nearest_lidar_m':.151,'obstacle_points':[(.119,.093)]}
            self.navigator.start_observation(self.pose,10.,snapshot,corner,(0.,0.,0.),0.)
            self.assertEqual(self.navigator.observation is not None,margin==.01)
            if margin==.01:self.assertGreater(self.navigator.sideways_velocity(self.pose,10.1,snapshot,corner)[2],0.)

    def test_vendor_side_preference_is_guarded_and_does_not_replace_clear_global_heading(self):
        import math
        from mediassist.lidar_reactive import LidarReactive
        snapshot,view=self.observation_scene()
        self.exploration.cfg['turn_clearance_m']=.2
        view={**view,'front_m':.045,'left_m':1.,'right_m':.14,
              'lateral_left_m':1.,'lateral_right_m':.02,'rotation_clear':False,
              'nearest_lidar_m':.145,'obstacle_points':[(.145,0.),(0.,-.145)]}
        policy=LidarReactive();policy.current=Mock(wraps=policy.current)
        self.navigator.lidar_reactive=policy;self.navigator.path=((0.,0.),(.3,0.))
        self.navigator.start_observation=Mock(return_value=False)
        self.navigator.start_sideways('vendor-escape',self.pose,10.,snapshot,view,dict(value=0.,age_sec=.01))
        policy.current.assert_called_once();self.navigator.start_observation.assert_not_called()
        self.assertIsNotNone(self.navigator.side)
        self.assertEqual(self.navigator.side[-1][0],0.);self.assertGreater(self.navigator.side[-1][1],0.)
        # A clear front must keep the Nav2 path's leftward heading.
        self.navigator=MappingNavigator(self.adapter,self.exploration)
        self.navigator.path=((0.,0.),(0.,.3))
        clear={**view,'front_m':1.,'obstacle_points':[],'nearest_lidar_m':1.,'rotation_clear':True}
        self.navigator.start_sideways('clear-path',self.pose,10.,snapshot,clear,dict(value=0.,age_sec=.01))
        self.assertIsNone(self.navigator.observation)
        self.assertIsNotNone(self.navigator.side)
        self.assertAlmostEqual(self.navigator.side[-1][0],0.)
        self.assertGreater(self.navigator.side[-1][1],0.)

if __name__=='__main__':unittest.main()
