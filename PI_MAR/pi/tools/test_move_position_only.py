"""Targeted position-only tracking/arrival regressions; no ROS, network or motors."""
import math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock,AsyncMock,patch
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'pi/tools')]
from mediassist.navigation_setup import nav2_params
from mediassist.ros_adapter import RosAdapter
from mediassist.contracts import StepError
from mediassist.navigation_zones import NavigationZones,rules

def params(**kw):
    base={'amcl':{'ros__parameters':{}},'velocity_smoother':{'ros__parameters':{}},
          'planner_server':{'ros__parameters':{'GridBased':{}}}}
    for name in ('local_costmap','global_costmap'):
        base[name]={name:{'ros__parameters':{'inflation_layer':{}}}}
    controller={'controller_server':{'ros__parameters':{'FollowPath':{
        'plugin':'dwb_core::DWBLocalPlanner','critics':['BaseObstacle','RotateToGoal']},
        'progress_checker':{},'general_goal_checker':{}}}}
    return nav2_params(base,controller,[],.6,kw.pop('angular',.8),mecanum=True,**kw)


class ProfileTests(unittest.TestCase):
    def test_final_heading_uses_forward_rpp_and_grid_path_without_a_dubins_end_loop(self):
        p=params(angular=1.2,fast_navigation=True,forward_only=True,position_then_spin=True,
                 footprint_planner=True,path_tracking=True)
        c=p['controller_server']['ros__parameters'];f=c['FollowPath']
        self.assertEqual(f['plugin'],'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
        self.assertFalse(f['allow_reversing']);self.assertTrue(f['use_collision_detection'])
        self.assertEqual(c['general_goal_checker']['xy_goal_tolerance'],.06)
        planner=p['planner_server']['ros__parameters']['GridBased']
        self.assertEqual(planner['plugin'],'nav2_smac_planner/SmacPlanner2D')
        self.assertTrue(planner['use_final_approach_orientation'])
        self.assertFalse(planner['allow_unknown'])
        self.assertGreater(planner['smoother']['w_smooth'],planner['smoother']['w_data'])
        self.assertEqual(p['velocity_smoother']['ros__parameters']['min_velocity'],[0.,0.,-1.2])
        self.assertGreaterEqual(p['behavior_server']['ros__parameters']['min_rotational_vel'],.1)
    def test_tracking_replaces_shim_dwb_and_dubins_terminal_heading(self):
        p=params(fast_navigation=True,footprint_planner=True,
            forward_only=True,position_only=True,path_tracking=True)
        c=p['controller_server']['ros__parameters'];f=c['FollowPath']
        self.assertEqual(f['plugin'],'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
        self.assertNotIn('critics',f);self.assertNotIn('primary_controller',f)
        self.assertFalse(f['allow_reversing']);self.assertTrue(f['use_collision_detection'])
        self.assertTrue(f['use_rotate_to_heading'])
        self.assertEqual(f['rotate_to_heading_min_angle'],.75)
        self.assertEqual(f['rotate_to_heading_angular_vel'],.8)
        self.assertEqual(f['max_angular_accel'],3.2)
        self.assertEqual(c['general_goal_checker']['xy_goal_tolerance'],.06)
        self.assertEqual(c['general_goal_checker']['yaw_goal_tolerance'],math.pi)
        planner=p['planner_server']['ros__parameters']['GridBased']
        self.assertEqual(planner['plugin'],'nav2_smac_planner/SmacPlanner2D')
        self.assertTrue(planner['use_final_approach_orientation']);self.assertFalse(planner['allow_unknown'])
        self.assertIn('mar_keepout',p['global_costmap']['global_costmap']['ros__parameters']['filters'])
        self.assertEqual(p['velocity_smoother']['ros__parameters']['min_velocity'][0],0.)

    def test_mapping_keeps_vendor_holonomic_controller(self):
        p=params(fast_mapping=True)
        f=p['controller_server']['ros__parameters']['FollowPath']
        self.assertEqual(f['plugin'],'dwb_core::DWBLocalPlanner')
        self.assertGreater(f['max_vel_y'],0);self.assertLess(f['min_vel_x'],0)

    def test_saved_map_yaw_is_independent_from_mapping_yaw(self):
        p=params(angular=1.2,fast_navigation=True,forward_only=True,position_only=True,path_tracking=True)
        self.assertEqual(p['controller_server']['ros__parameters']['FollowPath']['rotate_to_heading_angular_vel'],1.2)
        self.assertEqual(p['velocity_smoother']['ros__parameters']['max_velocity'],[.6,0.,1.2])
        self.assertEqual(p['velocity_smoother']['ros__parameters']['max_accel'][2],3.2)
        mapping=params(angular=1.2,fast_mapping=True)
        self.assertEqual(mapping['controller_server']['ros__parameters']['FollowPath']['max_vel_theta'],1.2)
        self.assertEqual(mapping['velocity_smoother']['ros__parameters']['max_accel'][2],2.4)
        self.assertEqual(mapping['local_costmap']['local_costmap']['ros__parameters']['footprint_padding'],.03)
        with self.assertRaises(ValueError):params(angular=1.21,fast_mapping=True)


class ArrivalTests(unittest.IsolatedAsyncioTestCase):
    def adapter(self):
        a=RosAdapter.__new__(RosAdapter)
        a.worker={'navigation_runtime':{'arrival_mode':'POSITION_ONLY','controller':'REGULATED_PURSUIT',
            'forward_only':True,'preserve_path_on_replan':True}}
        a.cfg={'endpoints':{'A':dict(x_m=1.,y_m=0.,yaw_rad=0.)},'max_speed_mps':.8,
            'motion':{'max_angular_speed_rps':.8,'allow_reverse':True,'allow_strafe':True}}
        self.pose=dict(x_m=.96,y_m=0.,yaw_rad=2.)
        a.telemetry=lambda:{'map_pose':{'fresh':True,'value':self.pose}}
        a._prerequisites=AsyncMock();a._wait_navigation_server=AsyncMock();a._pose=Mock()
        a._wait_stationary=AsyncMock();a._wait_arrival_pose=AsyncMock();a._finish_heading=AsyncMock()
        a.clear_navigation_velocity=Mock();a.navigation_speed_limit=Mock();a.lease_message=Mock()
        a.stop=AsyncMock(return_value=True);a.nav=Mock()
        a.node=Mock()
        a.future=AsyncMock(return_value=N(accepted=True,get_result_async=lambda:N(done=lambda:True,result=lambda:N(status=4))))
        return a

    async def test_one_navigate_goal_no_spin_actual_yaw_preserved(self):
        a=self.adapter()
        with patch.dict(sys.modules,{'nav2_msgs.action':N(NavigateToPose=N(Goal=lambda:N()))}):
            result=await a._move_one({'max_speed_mps':.6},'A')
        self.assertTrue(result['reached']);self.assertAlmostEqual(result['position_error_m'],.04)
        self.assertEqual(result['verified_pose']['yaw_rad'],2.)
        self.assertNotIn('yaw_error_rad',result)  # Never claim verified endpoint heading.
        self.assertEqual(a.nav.send_goal_async.call_count,1)
        a._finish_heading.assert_not_awaited()
        a._wait_arrival_pose.assert_awaited_once_with('A')

    async def test_heading_mode_tracks_one_goal_then_aligns_once_and_verifies_live_pose(self):
        a=self.adapter();a.worker['navigation_runtime']['arrival_mode']='POSITION_THEN_SPIN'
        async def align(*args):self.pose['yaw_rad']=0.
        a._finish_heading.side_effect=align
        with patch.dict(sys.modules,{'nav2_msgs.action':N(NavigateToPose=N(Goal=lambda:N()))}):
            result=await a._move_one({'max_speed_mps':.6},'A')
        self.assertEqual(a.nav.send_goal_async.call_count,1)
        a._finish_heading.assert_awaited_once_with({'max_speed_mps':.6},'A')
        a._wait_arrival_pose.assert_awaited_once_with('A')
        self.assertEqual(result['yaw_error_rad'],0.)
        self.assertEqual(result['verified_pose']['yaw_rad'],0.)

    def test_speed_zone_scales_turn_rate_with_forward_speed(self):
        a=self.adapter();a.navigation_velocity_output=Mock()
        a.zones=N(zones=[{}],directional_cap=lambda *args:.15)
        a.navigation_velocity=lambda:((.6,0.,.6),True)
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=lambda:N(linear=N(x=0.,y=0.),angular=N(z=0.)))}):
            a.publish_navigation_velocity(.6)
        msg=a.navigation_velocity_output.publish.call_args.args[0]
        self.assertAlmostEqual(msg.linear.x,.15);self.assertAlmostEqual(msg.angular.z,.15)

    def test_final_zone_clamp_preserves_ramped_curvature_and_zero_zone_stops_turn(self):
        for cap in (.15,0.):
            a=self.adapter();a.navigation_velocity_output=Mock()
            a.zones=N(zones=[{}],directional_cap=lambda *args:cap)
            a.navigation_velocity=lambda:((.6,0.,.6),True)
            a.navigation_resume_ramp=N(update=lambda *args:(.5,0.,.4))
            with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=lambda:N(linear=N(x=0.,y=0.),angular=N(z=0.)))}):
                a.publish_navigation_velocity(.6)
            msg=a.navigation_velocity_output.publish.call_args.args[0]
            self.assertAlmostEqual(msg.linear.x,cap)
            self.assertAlmostEqual(msg.angular.z,.12 if cap else 0.)

    def test_nearby_icon_is_not_a_substitute_for_arriving(self):
        a=self.adapter();self.pose['x_m']=.7
        with self.assertRaisesRegex(StepError,'ENDPOINT_FINAL_POSE_NOT_CONFIRMED'):a._verify_endpoint_arrival('A')

    def test_strict_heading_mode_still_requires_the_requested_heading(self):
        a=self.adapter();a.worker['navigation_runtime']['arrival_mode']='POSITION_THEN_SPIN'
        with self.assertRaisesRegex(StepError,'ENDPOINT_FINAL_POSE_NOT_CONFIRMED'):a._verify_endpoint_arrival('A')

    async def test_successful_spin_finishes_move_despite_post_turn_map_yaw_correction(self):
        a=self.adapter();a.worker['navigation_runtime']['arrival_mode']='POSITION_THEN_SPIN'
        self.pose['yaw_rad']=.24
        with patch.dict(sys.modules,{'nav2_msgs.action':N(NavigateToPose=N(Goal=lambda:N()))}):
            result=await a._move_one({'max_speed_mps':.6},'A')
        self.assertTrue(result['reached'])
        self.assertEqual(result['verified_pose']['yaw_rad'],.24)
        self.assertNotIn('yaw_error_rad',result)
        a._finish_heading.assert_awaited_once()
        a._wait_arrival_pose.assert_awaited_once()

    async def test_blocked_final_spin_does_not_finish_move_or_dispatch_another_turn(self):
        a=self.adapter();a.worker['navigation_runtime']['arrival_mode']='POSITION_THEN_SPIN'
        a._finish_heading.side_effect=StepError('FINAL_ALIGNMENT_BLOCKED')
        with patch.dict(sys.modules,{'nav2_msgs.action':N(NavigateToPose=N(Goal=lambda:N()))}):
            with self.assertRaisesRegex(StepError,'FINAL_ALIGNMENT_BLOCKED'):
                await a._move_one({'max_speed_mps':.6},'A')
        a._finish_heading.assert_awaited_once()

    def test_successful_turn_does_not_hide_position_drift_or_stale_localization(self):
        a=self.adapter();a.worker['navigation_runtime']['arrival_mode']='POSITION_THEN_SPIN'
        self.pose['x_m']=.7
        with self.assertRaisesRegex(StepError,'ENDPOINT_FINAL_POSE_NOT_CONFIRMED'):
            a._verify_endpoint_arrival('A',heading_completed=True)
        a.telemetry=lambda:{'map_pose':{'fresh':False}}
        with self.assertRaisesRegex(StepError,'LOCALIZATION_LOST_AT_GOAL'):
            a._verify_endpoint_arrival('A',heading_completed=True)

    def test_small_steering_is_not_amplified_but_rotation_can_start(self):
        a=self.adapter();a.navigation_velocity_output=Mock()
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=lambda:N(linear=N(x=0.,y=0.),angular=N(z=0.)))}):
            a.navigation_velocity=lambda:((.2,0.,.05),True);a.publish_navigation_velocity(.6)
            self.assertEqual(a.navigation_velocity_output.publish.call_args.args[0].angular.z,.05)
            a.navigation_velocity=lambda:((0.,0.,.06),True);a.publish_navigation_velocity(.6)
            self.assertEqual(a.navigation_velocity_output.publish.call_args.args[0].angular.z,.1)


class ZoneDirectionTests(unittest.TestCase):
    def zone(self,kind='KEEP_OUT'):
        z=NavigationZones.__new__(NavigationZones);z.se2=True
        z.footprint=dict(front=.109,rear=.104,left=.086,right=.086);z.radius=math.hypot(.109,.086)+.03
        z.zones=rules([dict(zone_type=kind,polygon=[[.3,-2],[.5,-2],[.5,2],[.3,2]],speed_limit_mps=.15)])
        return z
    def pose(self,yaw=0.,x=0.):return dict(fresh=True,value=dict(x_m=x,y_m=0.,yaw_rad=yaw))
    def test_parallel_or_away_motion_is_not_braked_for_a_zone_to_the_side(self):
        z=self.zone();p=self.pose(math.pi/2)
        self.assertLess(z.cap(p,.6),.6)
        self.assertEqual(z.directional_cap(p,.6,(.6,0.,0.)),.6)
        self.assertEqual(z.directional_cap(self.pose(math.pi),.6,(.6,0.,0.)),.6)
    def test_approaching_or_overlapping_zone_remains_protected(self):
        z=self.zone()
        self.assertGreater(z.directional_cap(self.pose(),.6,(.6,0.,0.)),0.)
        self.assertLess(z.directional_cap(self.pose(),.6,(.6,0.,0.)),.6)
        self.assertEqual(z.directional_cap(self.pose(x=.2),.6,(.6,0.,0.)),0.)
    def test_speed_zone_cap_still_applies(self):
        z=self.zone('SPEED')
        self.assertEqual(z.directional_cap(self.pose(x=.35),.6,(.6,0.,0.)),.15)

    def test_speed_zone_anticipates_entry_but_releases_when_moving_away(self):
        z=self.zone('SPEED')
        self.assertEqual(z.speed_cap(self.pose(x=0.),.6,(.6,0.,0.)),.15)
        self.assertEqual(z.speed_cap(self.pose(x=0.),.6,(.15,0.,0.)),.15)
        self.assertEqual(z.speed_cap(self.pose(x=0.),.6,(-.6,0.,0.)),.6)
        self.assertEqual(z.speed_cap(self.pose(x=.55),.6,(.6,0.,0.)),.15)  # Rear still inside.
        self.assertEqual(z.speed_cap(self.pose(x=.72),.6,(.15,0.,0.)),.6)  # Whole padded body out.
        self.assertEqual(z.speed_cap(self.pose(math.pi/2,x=0.),.6,(.6,0.,0.)),.6)

    def test_restricted_path_checks_three_centimetres_beyond_body_edge(self):
        z=self.zone();x=.3-z.footprint['front']-.03-.006
        z.validate_path([(x,0.,0.)])
        with self.assertRaisesRegex(StepError,'NAVIGATION_PATH_INTERSECTS_KEEPOUT'):
            z.validate_path([(x+.01,0.,0.)])


if __name__=='__main__':unittest.main()
