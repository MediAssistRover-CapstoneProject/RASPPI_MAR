"""Offline arrival and SE2 keepout regressions. No ROS or motor processes."""
import math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock,AsyncMock,patch
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools')]
from mediassist.contracts import StepError
from mediassist.ros_adapter import RosAdapter
from mediassist.navigation_zones import NavigationZones,rules,keepout_data,body_gap,compact_path
from mediassist.navigation_setup import nav2_params

class ArrivalTests(unittest.IsolatedAsyncioTestCase):
    def adapter(self,yaw=0.,status=4):
        a=RosAdapter.__new__(RosAdapter)
        a.cfg={'endpoints':{'A':dict(x_m=0.,y_m=0.,yaw_rad=1.)}}
        a.telemetry=lambda:{'map_pose':{'fresh':True,'value':dict(x_m=0.,y_m=0.,yaw_rad=yaw)}}
        a._wait_navigation_server=AsyncMock();a.clear_navigation_velocity=Mock();a.spin=Mock();a.node=Mock()
        a.stop=AsyncMock(return_value=True)
        result=N(done=lambda:True,result=lambda:N(status=status))
        handle=N(accepted=True,get_result_async=lambda:result)
        a.future=AsyncMock(return_value=handle)
        return a

    async def test_exactly_one_final_spin_goal(self):
        a=self.adapter()
        with patch.dict(sys.modules,{'nav2_msgs.action':N(Spin=N(Goal=lambda:N(time_allowance=N(sec=0))))}):
            self.assertTrue(await a._finish_heading({'max_speed_mps':.8},'A'))
        self.assertEqual(a.spin.send_goal_async.call_count,1)
        self.assertEqual(a.spin.send_goal_async.call_args.args[0].target_yaw,1.)
        self.assertEqual(a.stop.await_count,1)

    async def test_already_aligned_does_not_spin(self):
        a=self.adapter(yaw=.95)
        with patch.dict(sys.modules,{'nav2_msgs.action':N(Spin=N())}):
            self.assertFalse(await a._finish_heading({'max_speed_mps':.8},'A'))
        a.spin.send_goal_async.assert_not_called()

    async def test_blocked_spin_stops_without_a_second_attempt(self):
        a=self.adapter(status=6)
        with patch.dict(sys.modules,{'nav2_msgs.action':N(Spin=N(Goal=lambda:N(time_allowance=N(sec=0))))}):
            with self.assertRaisesRegex(StepError,'FINAL_ALIGNMENT_BLOCKED'):
                await a._finish_heading({'max_speed_mps':.8},'A')
        self.assertEqual(a.spin.send_goal_async.call_count,1);self.assertEqual(a.stop.await_count,1)

    async def test_no_alignment_before_arriving(self):
        a=self.adapter();a.cfg['endpoints']['A']['x_m']=.2
        with patch.dict(sys.modules,{'nav2_msgs.action':N(Spin=N())}):
            with self.assertRaisesRegex(StepError,'ENDPOINT_POSITION_NOT_CONFIRMED'):
                await a._finish_heading({'max_speed_mps':.8},'A')
        a.spin.send_goal_async.assert_not_called()

    async def test_blocked_rotation_sweep_is_rejected_before_spin(self):
        a=self.adapter();a.zones=N(validate_path=Mock(side_effect=StepError('NAVIGATION_PATH_INTERSECTS_KEEPOUT')))
        with patch.dict(sys.modules,{'nav2_msgs.action':N(Spin=N())}):
            with self.assertRaisesRegex(StepError,'FINAL_ALIGNMENT_BLOCKED_BY_ZONE'):
                await a._finish_heading({'max_speed_mps':.8},'A')
        a.spin.send_goal_async.assert_not_called()

class SE2Tests(unittest.TestCase):
    def test_long_path_preview_preserves_a_detour_instead_of_cutting_corners(self):
        points=[(-1+i*.001,0.,0.) for i in range(1001)]
        self.assertEqual(len(compact_path(points)),2)
        points=[(-.4,.5,math.pi/2),(-.4,1.4,math.pi/2),(-.4,1.4,0.),
                (1.4,1.4,0.),(1.4,1.4,-math.pi/2),(1.4,.5,-math.pi/2)]
        zone=NavigationZones.__new__(NavigationZones);zone.radius=.15
        zone.zones=rules([dict(zone_type='KEEP_OUT',polygon=[[0,0],[1,0],[1,1],[0,1]])])
        preview=zone.preview_path(points);zone.validate_path(preview)
        self.assertGreater(len(preview),2)

    def test_speed_zone_updates_controller_and_final_output_then_releases_on_exit(self):
        a=RosAdapter.__new__(RosAdapter)
        a.cfg={'max_speed_mps':.8,'motion':{'allow_reverse':True,'max_reverse_speed_mps':.8,'allow_strafe':False,'max_angular_speed_rps':.8}}
        a.node=Mock();a.navigation_velocity=lambda:((.8,0.,.3),True)
        a.navigation_velocity_output=Mock();a.navigation_speed_output=Mock()
        a.zones=NavigationZones.__new__(NavigationZones);a.zones.radius=.2
        a.zones.zones=rules([dict(zone_type='SPEED',polygon=[[0,0],[1,0],[1,1],[0,1]],speed_limit_mps=.15)])
        p={'fresh':True,'value':dict(x_m=.5,y_m=.5,yaw_rad=0.)};a.telemetry=lambda:{'map_pose':p}
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=lambda:N(linear=N(x=0.,y=0.),angular=N(z=0.))),
                'nav2_msgs.msg':N(SpeedLimit=lambda:N(header=N(stamp=None)))}):
            a.publish_navigation_velocity(.8);a.publish_navigation_velocity(.8)
            self.assertEqual(a.navigation_speed_output.publish.call_count,1)
            self.assertFalse(a.navigation_speed_output.publish.call_args.args[0].percentage)
            self.assertEqual(a.navigation_speed_output.publish.call_args.args[0].speed_limit,.15)
            self.assertEqual(a.navigation_velocity_output.publish.call_args.args[0].linear.x,.15)
            p['value']['x_m']=5.;a.publish_navigation_velocity(.8)
            self.assertEqual(a.navigation_speed_output.publish.call_args.args[0].speed_limit,.8)
            a.publish_navigation_velocity(.8,rotation_only=True)
            self.assertEqual(a.navigation_velocity_output.publish.call_args.args[0].linear.x,0.)

    def zones(self):
        a=NavigationZones.__new__(NavigationZones);a.se2=True
        a.footprint=dict(front=.12,rear=.12,left=.1,right=.1);a.radius=math.hypot(.12,.1)+.03
        a.zones=rules([dict(zone_type='KEEP_OUT',polygon=[[.2,.18],[1.,.18],[1.,.8],[.2,.8]])])
        a.zone_grid=dict(width=120,height=100,resolution=.025,origin=[-1.,-.5,0.])
        a.zone_grid['data']=keepout_data(a.zone_grid,a.zones,0.)
        return a

    def test_oriented_body_can_pass_where_circumscribed_disc_would_block(self):
        a=self.zones();a.validate_path([(-.2,0.,0.),(1.4,0.,0.)])
        self.assertLess(body_gap(dict(x_m=.5,y_m=0.,yaw_rad=0.),a.footprint,a.zones[0]['points']),a.radius)
        with self.assertRaisesRegex(StepError,'INTERSECTS_KEEPOUT'):
            a.validate_path([(-.2,0.,0.),(.5,.25,0.),(1.4,0.,0.)])

    def test_se2_halo_never_turns_body_clearance_into_a_hard_disc(self):
        a=self.zones();mask=keepout_data(a.zone_grid,a.zones,0.,.25,a.radius+.04)
        binary=keepout_data(a.zone_grid,a.zones,0.)
        self.assertEqual([x==100 for x in mask],[x==100 for x in binary])
        self.assertTrue(any(x==98 for x in mask))

    def test_profile_separates_position_arrival_and_full_body_planning(self):
        base={'amcl':{'ros__parameters':{}},'velocity_smoother':{'ros__parameters':{}},
              'planner_server':{'ros__parameters':{'GridBased':{}}}}
        for n in ('local_costmap','global_costmap'):base[n]={n:{'ros__parameters':{'inflation_layer':{}}}}
        ctl={'controller_server':{'ros__parameters':{'FollowPath':{'plugin':'dwb_core::DWBLocalPlanner',
             'critics':['BaseObstacle','RotateToGoal']},'progress_checker':{},'general_goal_checker':{}}}}
        params=nav2_params(base,ctl,[[.12,.1],[-.12,.1],[-.12,-.1],[.12,-.1]],.8,.8,
            mecanum=True,fast_navigation=True,position_then_spin=True,footprint_planner=True)
        follow=params['controller_server']['ros__parameters']['FollowPath']
        self.assertNotIn('RotateToGoal',follow['critics']);self.assertEqual(follow['GoalAlign.scale'],0.)
        self.assertEqual(follow['max_vel_y'],0.)
        self.assertEqual(params['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance'],math.pi)
        self.assertEqual(params['planner_server']['ros__parameters']['GridBased']['plugin'],'nav2_smac_planner/SmacPlannerHybrid')
        self.assertFalse(params['global_costmap']['global_costmap']['ros__parameters']['rolling_window'])
        self.assertIn('mar_keepout_inflation',params['global_costmap']['global_costmap']['ros__parameters']['filters'])

if __name__=='__main__':unittest.main()
