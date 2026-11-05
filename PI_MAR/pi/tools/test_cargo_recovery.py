"""Cargo escape and directional swept braking. No ROS nodes, network or motors."""
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, AsyncMock, patch
from types import SimpleNamespace as N
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'pi')]
from mediassist.contracts import StepError
from mediassist.cargo_recovery import CargoRecovery, maneuver_points, maneuver_clear, turn_options, padding_escape_clear, blocked_motion_reason, cached_body_clear
from mediassist.motion_braking import limit_velocity, swept_turn_clear

BODY=dict(front=.109,rear=.204,left=.086,right=.086)
PROFILE=dict(vehicle='rigid',footprint=BODY,stop_margin_m=.02,scan_max_age_sec=.4,
             lidar_stop_distance_m=.05,speed_mps=.6)


class SweepTests(unittest.TestCase):
    def test_free_space_cache_matches_exact_geometry_and_never_hides_changed_or_unknown_cells(self):
        from mediassist.footprint import body_clear
        grid=dict(width=120,height=120,resolution=.025,origin=[0.,0.,.1],data=bytes(14400))
        for value in (0,100,255):
            raw=bytearray(14400);raw[60*120+64]=value;grid['data']=bytes(raw)
            for x,y,yaw in ((1.5,1.5,0.),(1.5,1.5,.7),(.01,.01,0.)):
                pose=dict(x_m=x,y_m=y,yaw_rad=yaw)
                self.assertEqual(cached_body_clear(grid,pose,BODY),body_clear(grid,pose,BODY))

    def test_long_opposite_turn_is_not_wrapped_back_to_the_blocked_short_arc(self):
        origin=dict(x_m=1.,y_m=1.,yaw_rad=0.)
        short,long=turn_options(0.,1.)
        self.assertAlmostEqual(short,1.);self.assertAlmostEqual(long,1.-2*math.pi)
        points=maneuver_points(origin,0.,1.,turn_delta=long)
        self.assertLess(points[-1][2],-math.pi)
        self.assertTrue(all(b[2]<=a[2] for a,b in zip(points,points[1:])))

    def test_padding_escape_moves_away_without_crossing_a_physical_or_new_obstacle(self):
        grid=dict(width=100,height=100,resolution=.02,origin=[0.,0.,0.],data=bytes(10000))
        origin=dict(x_m=1.,y_m=1.,yaw_rad=0.)
        points=maneuver_points(origin,.06,0.)
        obstacle=[(1.13,1.)]
        self.assertFalse(maneuver_clear(grid,points,BODY,obstacle))
        self.assertTrue(padding_escape_clear(grid,points,BODY,obstacle))
        self.assertFalse(padding_escape_clear(grid,maneuver_points(origin,-.06,0.),BODY,obstacle))
        self.assertFalse(padding_escape_clear(grid,points,BODY,obstacle+[(.77,1.)]))
        self.assertFalse(padding_escape_clear(grid,maneuver_points(origin,0.,1.),BODY,obstacle))
        zones=Mock();zones.validate_path.side_effect=StepError('NAVIGATION_PATH_INTERSECTS_KEEPOUT')
        self.assertFalse(padding_escape_clear(grid,points,BODY,obstacle,zones))

    def test_side_step_preserves_heading_and_detects_cargo_side_collision(self):
        grid=dict(width=100,height=100,resolution=.02,origin=[0.,0.,0.],data=bytes(10000))
        origin=dict(x_m=1.,y_m=1.,yaw_rad=0.)
        points=maneuver_points(origin,0.,0.,lateral=.06)
        self.assertAlmostEqual(points[-1][1],1.06)
        self.assertTrue(maneuver_clear(grid,points,BODY,[]))
        self.assertFalse(maneuver_clear(grid,points,BODY,[(.82,1.14)]))

    def test_gate_only_triggers_escape_for_a_confirmed_collision_stop(self):
        gate=dict(reason='TURN_ENVELOPE_BLOCKED',applied_velocity=[0.,0.,0.])
        self.assertEqual(blocked_motion_reason({'gate':{'value':gate}}),'TURN_ENVELOPE_BLOCKED')
        self.assertIsNone(blocked_motion_reason({'gate':{'value':{**gate,'applied_velocity':[.05,0.,0.]}}}))
        self.assertIsNone(blocked_motion_reason({'gate':{'value':{**gate,'reason':'NO_LEASE'}}}))

    def test_front_obstacle_in_360_circle_does_not_block_a_small_safe_turn(self):
        view=dict(valid=True,nearest_lidar_m=.18,rotation_clear=False,obstacle_points=[(.18,0.)])
        command,reason=limit_velocity((0.,0.,.2),view,.1,PROFILE)
        self.assertEqual(command,(0.,0.,.2));self.assertIsNone(reason)

    def test_tail_sweep_checks_turn_direction_and_reduces_unsafe_speed(self):
        points=[(-.18,-.13)]
        self.assertFalse(swept_turn_clear((0.,0.,.8),points,BODY,.02,.1))
        self.assertTrue(swept_turn_clear((0.,0.,-.2),points,BODY,.02,.1))
        view=dict(valid=True,nearest_lidar_m=.22,rotation_clear=False,obstacle_points=points)
        command,_=limit_velocity((0.,0.,.8),view,.1,PROFILE)
        self.assertLess(command[2],.8)
        self.assertTrue(command[2]==0. or command[2]>=.1)

    def test_blocking_a_curve_never_discards_yaw_and_keeps_translation(self):
        view=dict(valid=True,nearest_lidar_m=.2,rotation_clear=False,obstacle_points=[(.15,.15)])
        command,_=limit_velocity((.2,0.,.8),view,.1,PROFILE)
        if command[0]:self.assertAlmostEqual(command[2]/command[0],4.)
        else:self.assertEqual(command,(0.,0.,0.))
        missing={k:v for k,v in view.items() if k!='obstacle_points'}
        self.assertEqual(limit_velocity((.2,0.,.8),missing,.1,PROFILE)[0],(0.,0.,0.))

    def test_stale_scan_and_obstacle_inside_body_still_stop(self):
        view=dict(valid=True,nearest_lidar_m=.2,rotation_clear=False,obstacle_points=[(-.15,0.)])
        self.assertEqual(limit_velocity((0.,0.,.2),view,.1,PROFILE)[0],(0.,0.,0.))
        self.assertEqual(limit_velocity((0.,0.,.2),view,1.,PROFILE)[0],(0.,0.,0.))

    def test_reverse_turn_sweep_includes_cargo_and_keepout(self):
        origin=dict(x_m=1.,y_m=1.,yaw_rad=0.)
        points=maneuver_points(origin,.10,math.pi/2)
        self.assertAlmostEqual(points[-1][0],.9);self.assertAlmostEqual(points[-1][2],math.pi/2)
        grid=dict(width=100,height=100,resolution=.02,origin=[0.,0.,0.],data=bytes(10000))
        self.assertTrue(maneuver_clear(grid,points,BODY,[]))
        self.assertFalse(maneuver_clear(grid,points,BODY,[(.69,1.)]))
        zones=Mock();zones.validate_path.side_effect=StepError('NAVIGATION_PATH_INTERSECTS_KEEPOUT')
        self.assertFalse(maneuver_clear(grid,points,BODY,[],zones))
        with self.assertRaises(StepError):maneuver_points(origin,.11,0.)


class RecoveryTests(unittest.IsolatedAsyncioTestCase):
    def adapter(self):
        a=CargoRecovery()
        a.worker={'navigation_runtime':{'geometry':{'profile':'cargo'}}}
        a.cfg={'motion':{'allow_reverse':True,'allow_strafe':True,'chassis_model':'MENTORPI_M1_MECANUM'},'endpoints':{'A':dict(x_m=0.,y_m=1.,yaw_rad=0.)}}
        a._wait_stationary=AsyncMock();a._cargo_pose=Mock(return_value=({},dict(x_m=0.,y_m=0.,yaw_rad=0.)))
        a._cargo_reverse=AsyncMock();a._cargo_translate=AsyncMock();a._cargo_rotate=AsyncMock();a.node=Mock()
        a._cargo_clear=lambda p,d,y,**kw:(True,maneuver_points(p,d,y,**kw))
        return a

    async def test_controller_abort_replans_once_inside_same_step(self):
        a=self.adapter();a._move_one=AsyncMock(side_effect=[StepError('NAV_ABORTED'),{'reached':True}])
        a.telemetry=Mock(return_value={});a._navigation_context=Mock(return_value=True)
        a._prerequisites=AsyncMock();a._plan_with_cargo_recovery=AsyncMock()
        params={'max_speed_mps':.4};meta={'job_id':'same','job_step_id':'same-step'}
        self.assertTrue((await a._move_with_path_recovery(params,'A',meta))['reached'])
        a._plan_with_cargo_recovery.assert_awaited_once_with(params,'A',meta)
        self.assertEqual(a._move_one.await_count,2)

    async def test_small_arrival_error_replans_once_without_faking_success(self):
        a=self.adapter();a._move_one=AsyncMock(side_effect=[StepError('ENDPOINT_POSITION_NOT_CONFIRMED'),{'reached':True}])
        a.telemetry=Mock(return_value={'map_pose':{'fresh':True,'value':dict(x_m=0.,y_m=.84)}})
        a._navigation_context=Mock(return_value=True);a._prerequisites=AsyncMock();a._plan_with_cargo_recovery=AsyncMock()
        params={'max_speed_mps':.4};meta={'job_id':'same','job_step_id':'same-step'}
        self.assertTrue((await a._move_with_path_recovery(params,'A',meta))['reached'])
        a._plan_with_cargo_recovery.assert_awaited_once_with(params,'A',meta)
        self.assertEqual(a._move_one.await_count,2)

    async def test_large_or_stale_arrival_error_never_starts_correction(self):
        for fresh,y,error in ((True,.7,'ENDPOINT_POSITION_NOT_CONFIRMED'),(False,.84,'LOCALIZATION_LOST')):
            a=self.adapter();a._move_one=AsyncMock(side_effect=StepError('ENDPOINT_POSITION_NOT_CONFIRMED'))
            a.telemetry=Mock(return_value={'map_pose':{'fresh':fresh,'value':dict(x_m=0.,y_m=y)}})
            a._navigation_context=Mock(return_value=True);a._plan_with_cargo_recovery=AsyncMock()
            with self.assertRaisesRegex(StepError,error):await a._move_with_path_recovery({},'A',{})
            a._plan_with_cargo_recovery.assert_not_awaited()

    async def test_arrival_correction_failure_is_terminal(self):
        a=self.adapter();a._move_one=AsyncMock(side_effect=StepError('ENDPOINT_POSITION_NOT_CONFIRMED'))
        a.telemetry=Mock(return_value={'map_pose':{'fresh':True,'value':dict(x_m=0.,y_m=.84)}})
        a._navigation_context=Mock(return_value=True);a._prerequisites=AsyncMock();a._plan_with_cargo_recovery=AsyncMock()
        with self.assertRaisesRegex(StepError,'ENDPOINT_POSITION_NOT_CONFIRMED'):await a._move_with_path_recovery({},'A',{})
        self.assertEqual(a._move_one.await_count,2)

    async def test_stopped_pose_can_settle_before_arrival_is_rejected(self):
        from mediassist.ros_adapter import RosAdapter
        a=RosAdapter.__new__(RosAdapter);a.cfg={'endpoints':{'A':dict(x_m=0.,y_m=0.)}}
        a.lease_message=Mock();a.node=Mock();a._navigation_context=Mock(return_value=True)
        poses=iter([.16,.07,.07,.07,.07])
        a.telemetry=lambda:{'map_pose':{'fresh':True,'value':dict(x_m=next(poses,.07),y_m=0.,yaw_rad=0.)}}
        clock=iter([0.,0.,0.,.1,.1,.2,.5,.6])
        with patch('mediassist.ros_adapter.time.monotonic',side_effect=lambda:next(clock,1.)),patch('mediassist.ros_adapter.asyncio.sleep',new=AsyncMock()):
            await a._wait_arrival_pose('A')
        a.node.get_logger().warning.assert_not_called()

    async def test_repeated_obstacles_are_bounded_not_an_infinite_retry(self):
        a=self.adapter();a._move_one=AsyncMock(side_effect=StepError('NAV_ABORTED'))
        a.telemetry=Mock(return_value={});a._navigation_context=Mock(return_value=True)
        a._prerequisites=AsyncMock();a._plan_with_cargo_recovery=AsyncMock()
        with self.assertRaisesRegex(StepError,'NAV_ABORTED'):await a._move_with_path_recovery({},'A',{})
        self.assertEqual(a._move_one.await_count,3);self.assertEqual(a._plan_with_cargo_recovery.await_count,2)

    async def test_two_dynamic_blockages_can_replan_inside_one_step(self):
        a=self.adapter();a._move_one=AsyncMock(side_effect=[StepError('NAV_ABORTED'),StepError('NAV_ABORTED'),{'reached':True}])
        a.telemetry=Mock(return_value={});a._navigation_context=Mock(return_value=True)
        a._prerequisites=AsyncMock();a._plan_with_cargo_recovery=AsyncMock()
        self.assertTrue((await a._move_with_path_recovery({},'A',{}))['reached'])
        self.assertEqual(a._move_one.await_count,3);self.assertEqual(a._plan_with_cargo_recovery.await_count,2)

    async def test_repeated_gate_blockage_never_repeats_physical_cargo_escape(self):
        a=self.adapter();a._move_one=AsyncMock(side_effect=[StepError('NAV_GATE_BLOCKED'),StepError('NAV_GATE_BLOCKED'),{'reached':True}])
        a.telemetry=Mock(return_value={});a._navigation_context=Mock(return_value=True);a._prerequisites=AsyncMock()
        async def replan(*args,**kw):
            if kw.get('force_escape'):a.cargo_escape_used=True
        a._plan_with_cargo_recovery=AsyncMock(side_effect=replan)
        self.assertTrue((await a._move_with_path_recovery({},'A',{}))['reached'])
        self.assertEqual(a._plan_with_cargo_recovery.await_args_list[0].kwargs,{'force_escape':True})
        self.assertEqual(a._plan_with_cargo_recovery.await_args_list[1].kwargs,{})

    async def test_cancel_sensor_loss_and_unconfirmed_stop_never_retry_motion(self):
        for code in ('NAV_CANCELLED','LOCALIZATION_LOST','SAFETY_GATE_LOST','GOAL_STOP_UNCONFIRMED'):
            a=self.adapter();a._move_one=AsyncMock(side_effect=StepError(code));a._plan_with_cargo_recovery=AsyncMock()
            with self.assertRaisesRegex(StepError,code):await a._move_with_path_recovery({},'A',{})
            a._plan_with_cargo_recovery.assert_not_awaited()

    async def test_lost_pose_after_abort_prevents_retry(self):
        a=self.adapter();a._move_one=AsyncMock(side_effect=StepError('NAV_ABORTED'))
        a.telemetry=Mock(return_value={});a._navigation_context=Mock(return_value=False)
        a._plan_with_cargo_recovery=AsyncMock()
        with self.assertRaisesRegex(StepError,'LOCALIZATION_LOST'):await a._move_with_path_recovery({},'A',{})
        a._plan_with_cargo_recovery.assert_not_awaited()

    async def test_cargo_translation_uses_direct_bearing_without_overwriting_final_heading(self):
        from mediassist.ros_adapter import RosAdapter
        a=RosAdapter.__new__(RosAdapter)
        target=dict(x_m=1.,y_m=0.,yaw_rad=math.pi/2)
        a.cfg={'endpoints':{'A':target},'ros':{'map_frame':'map'},
               'navigation':{'planner_id':'GridBased','planning_timeout_sec':8.}}
        a.worker={'navigation_runtime':{'arrival_mode':'POSITION_THEN_SPIN','full_footprint_search':True,
                                      'controller':'REGULATED_PURSUIT'}}
        a.telemetry=lambda:{'map_pose':{'fresh':True,'value':dict(x_m=0.,y_m=0.,yaw_rad=0.)}}
        a._check_endpoint_footprint=Mock();a._wait_navigation_server=AsyncMock();a._prerequisites=AsyncMock()
        a.stop=AsyncMock(return_value=True);a.node=Mock();a.zones=None
        def pose():return N(header=N(frame_id='map',stamp=None),pose=N(position=N(x=0.,y=0.),orientation=N(x=0.,y=0.,z=0.,w=1.)))
        path=N(header=N(frame_id='map'),poses=[pose(),pose()]);path.poses[-1].pose.position.x=1.
        handle=N(accepted=True,get_result_async=lambda:'result')
        result=N(status=4,result=N(path=path))
        a.planner=Mock();a.future=AsyncMock(side_effect=[handle,result])
        with patch.dict(sys.modules,{'nav2_msgs.action':N(ComputePathToPose=N(Goal=lambda:N())),
                                    'geometry_msgs.msg':N(PoseStamped=pose)}):
            await a._plan({'max_speed_mps':.4},'A')
        goal=a.planner.send_goal_async.call_args.args[0].goal
        self.assertAlmostEqual(goal.pose.orientation.z,0.)
        self.assertEqual(target['yaw_rad'],math.pi/2)

    async def test_successful_direct_plan_never_runs_escape(self):
        a=self.adapter();a._plan=AsyncMock(return_value='path')
        self.assertEqual(await a._plan_with_cargo_recovery({},'A',{}),'path')
        a._cargo_reverse.assert_not_awaited();a._cargo_rotate.assert_not_awaited()

    async def test_no_movement_until_a_virtual_start_route_is_found(self):
        a=self.adapter();a._plan=AsyncMock(side_effect=StepError('PATH_UNAVAILABLE'))
        with self.assertRaises(StepError):await a._plan_with_cargo_recovery({},'A',{})
        a._cargo_reverse.assert_not_awaited();a._cargo_rotate.assert_not_awaited()
        self.assertLessEqual(a._plan.await_count,7)

    async def test_reverse_and_rotate_once_then_replan_actual_pose(self):
        a=self.adapter();a._plan=AsyncMock(side_effect=[StepError('PATH_UNAVAILABLE'),'virtual','actual'])
        a._cargo_clear=lambda p,d,y,**kw:(d>0.,maneuver_points(p,d,y,**kw))
        self.assertEqual(await a._plan_with_cargo_recovery({},'A',{}),'actual')
        a._cargo_reverse.assert_awaited_once_with(.06,{})
        a._cargo_rotate.assert_awaited_once()
        self.assertTrue(a._plan.await_args_list[1].kwargs['probe'])
        self.assertEqual(a._plan.await_args_list[-1].kwargs,{})

    async def test_bare_and_endpoint_group_do_not_run_physical_recovery(self):
        for bare,params in ((True,{}),(False,{'endpoint_group_id':'group'})):
            a=self.adapter();a._plan=AsyncMock(side_effect=StepError('PATH_UNAVAILABLE'))
            if bare:a.worker['navigation_runtime']['geometry']['profile']='bare'
            with self.assertRaises(StepError):await a._plan_with_cargo_recovery(params,'A',{})
            a._cargo_reverse.assert_not_awaited();a._cargo_rotate.assert_not_awaited()

    async def test_final_heading_checks_and_retreats_before_issuing_spin(self):
        a=self.adapter();a.cfg['endpoints']['A']=dict(x_m=0.,y_m=0.,yaw_rad=1.)
        a._cargo_clear=lambda p,d,y,**kw:(d>0.,maneuver_points(p,d,y,**kw))
        await a._cargo_prepare_final_heading({},'A')
        a._cargo_reverse.assert_awaited_once_with(.06,{})
        a._cargo_rotate.assert_not_awaited()
        self.assertTrue(a.cargo_escape_used)

    async def test_final_retreat_must_not_leave_arrival_tolerance_or_repeat(self):
        for repeated in (False,True):
            a=self.adapter();a.cfg['endpoints']['A']=dict(x_m=.08,y_m=0.,yaw_rad=1.)
            a._cargo_clear=lambda p,d,y,**kw:(d>0.,maneuver_points(p,d,y,**kw))
            a.cargo_escape_used=repeated
            with self.assertRaises(StepError):await a._cargo_prepare_final_heading({},'A')
            a._cargo_reverse.assert_not_awaited()

    async def test_gate_stall_uses_a_side_step_then_replans_from_the_real_pose(self):
        a=self.adapter();a._plan=AsyncMock(side_effect=['virtual','actual'])
        a._cargo_clear=lambda p,d,y,**kw:(kw.get('lateral',0)>0,maneuver_points(p,d,y,**kw))
        self.assertEqual(await a._plan_with_cargo_recovery({},'A',{},force_escape=True),'actual')
        a._cargo_translate.assert_awaited_once_with(0.,.06,{})
        self.assertEqual(a._plan.await_args_list[-1].kwargs,{})

    async def test_final_alignment_selects_the_other_direction_before_moving_the_body(self):
        a=self.adapter();a.cfg['endpoints']['A']=dict(x_m=0.,y_m=0.,yaw_rad=1.)
        a._cargo_clear=lambda p,d,y,**kw:(kw.get('turn_delta',0)<-math.pi,maneuver_points(p,d,y,**kw))
        await a._cargo_prepare_final_heading({},'A')
        self.assertLess(a.cargo_final_turn_sign,0.)
        a._cargo_translate.assert_not_awaited();a._cargo_reverse.assert_not_awaited()


if __name__=='__main__':unittest.main()
