"""Regression for lateral Nav2 motion, late localization and frontier re-observation."""
import copy,math,sys,threading,unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock,AsyncMock,patch
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools'),str(ROOT/'pi/tools')]
from mediassist.navigation_setup import nav2_params
from mediassist.ros_adapter import RosAdapter
from mediassist.autonomous_mapping import nav2_assist
from mediassist.mapping_control import HolonomicRamp
from mediassist.localization import bootstrap_verdict,QualityGate
from mediassist.frontier_mapping import plan
from mediassist.worker_config import camera_config
import test_localization as loc
import test_frontier_mapping as frontier

class MecanumTests(unittest.TestCase):
    def test_holonomic_nav2_profile_is_opt_in_and_keeps_smoother_y_limits_consistent(self):
        base={'amcl':{'ros__parameters':{}},'velocity_smoother':{'ros__parameters':{}},
              'planner_server':{'ros__parameters':{'GridBased':{}}}}
        for name in ('local_costmap','global_costmap'):
            base[name]={name:{'ros__parameters':{'robot_radius':.3,'inflation_layer':{}}}}
        ctl={'controller_server':{'ros__parameters':{'FollowPath':{'plugin':'dwb_core::DWBLocalPlanner','critics':['BaseObstacle']},'progress_checker':{},'general_goal_checker':{}}}}
        polygon=[[.12,.1],[-.12,.1],[-.12,-.1],[.12,-.1]]
        for enabled in (False,True):
            params=nav2_params(base,ctl,polygon,.18,.6,mecanum=enabled)
            follow=params['controller_server']['ros__parameters']['FollowPath']
            smooth=params['velocity_smoother']['ros__parameters']
            self.assertEqual(follow['max_vel_y'],.18 if enabled else 0.)
            self.assertEqual(smooth['max_velocity'][1],follow['max_vel_y'])
            self.assertEqual(smooth['min_velocity'][1],follow['min_vel_y'])
            if enabled:self.assertGreater(follow['acc_lim_y'],0.)
        self.assertNotIn('max_vel_y',ctl['controller_server']['ros__parameters']['FollowPath'])
    def test_runtime_preset_preserves_identity_geometry_secrets_and_manual_settings(self):
        from configure_mecanum_runtime import tune
        cfg={'robot_id':'robot','mqtt':{'host':'broker'},'mapping':{'autonomy':frontier.profile(),
             'camera':{'enabled':True},'control_source':'gamepad','upload_base_url':'https://example.org'}}
        cfg['mapping']['autonomy']['footprint']={'front':.12,'rear':.12,'left':.1,'right':.1}
        hw={'max_speed_mps':.25,'motion':dict(chassis_model='MENTORPI_M1_MECANUM',allow_strafe=True,
             max_angular_speed_rps=.8,commissioned=True,cart_footprint_validated=True,watchdog_validated=True)}
        before=copy.deepcopy(cfg);hardware=copy.deepcopy(hw);result=tune(cfg,hw)
        self.assertEqual(cfg,before);self.assertEqual(hw,hardware)
        self.assertEqual(result['mqtt'],before['mqtt']);self.assertEqual(result['mapping']['control_source'],'gamepad')
        self.assertEqual(result['mapping']['autonomy']['footprint'],before['mapping']['autonomy']['footprint'])
        self.assertEqual(result['mapping']['autonomy']['speed_mps'],.18)
        self.assertFalse(result['mapping']['camera']['mqtt_preview_enabled'])
        with self.assertRaises(ValueError):tune(cfg,hw,speed=.3)
    def test_nav2_lateral_velocity_survives_adapter_with_vector_cap(self):
        a=object.__new__(RosAdapter);a.lock=threading.Lock();a.nav_velocity_sample=((.2,.2,.3),10.)
        a.cfg={'max_speed_mps':.18,'motion':{'allow_strafe':True,'max_angular_speed_rps':.6}}
        a.navigation_velocity_output=Mock()
        def twist():return N(linear=N(x=0.,y=0.),angular=N(z=0.))
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=twist)}),patch('mediassist.ros_adapter.time.monotonic',return_value=10.1):
            self.assertTrue(a.publish_navigation_velocity(.18))
            msg=a.navigation_velocity_output.publish.call_args.args[0]
            self.assertGreater(msg.linear.y,0);self.assertAlmostEqual(math.hypot(msg.linear.x,msg.linear.y),.18)
        with patch('mediassist.ros_adapter.time.monotonic',return_value=10.5):
            self.assertEqual(a.navigation_velocity(),((0.,0.,0.),False))
    def test_front_wall_does_not_cancel_clear_lateral_escape_but_side_wall_stops_it(self):
        cfg=frontier.profile();view={'front_m':.05,'lateral_left_m':1.,'lateral_right_m':.05}
        self.assertEqual(nav2_assist((.1,.1,.2),view,cfg),(0.,.1,.2))
        self.assertEqual(nav2_assist((0.,-.1,0.),view,cfg),(0.,0.,0.))
        x,y,_=nav2_assist((1.,1.,0.),{**view,'front_m':1.},cfg)
        self.assertAlmostEqual(math.hypot(x,y),cfg['speed_mps'])
    def test_new_obstacle_or_zero_stops_the_affected_axis_without_ramp_coasting(self):
        r=HolonomicRamp()
        for i in range(30):r.update((.18,.05,.3),i*.05)
        x,y,z=r.update((0.,.05,.3),1.6)
        self.assertEqual(x,0.);self.assertGreater(y,0.)
        self.assertEqual(r.update((0.,0.,0.),1.65),(0.,0.,0.))
    def test_verified_localization_can_recover_after_bootstrap_timeout(self):
        gate=QualityGate(loc.cfg());status=None
        for i in range(10):status=gate.update(65+i*.5,loc.evidence(i))
        self.assertTrue(status['ready'])
        self.assertTrue(bootstrap_verdict(status,True,False,70,60,'WAITING')['ready'])
        stale=loc.evidence(20);stale['scan_age_sec']=2
        status=gate.update(71,stale)
        self.assertFalse(bootstrap_verdict(status,True,False,71,60,'WAITING')['ready'])
        self.assertFalse(bootstrap_verdict({**status,'ready':True},False,False,71,60,'WAITING')['ready'])
    def test_successful_observation_exclusion_does_not_cut_off_deeper_candidates(self):
        c=frontier.profile();c['frontier_controller']='nav2';snapshot=frontier.room(True)
        result=plan(snapshot,frontier.POSE,c);self.assertEqual(result['state'],'FRONTIERS_PRESENT')
        goal=result['goal'];c['_excluded_goals']=[(goal['x_m'],goal['y_m'])]
        other=plan(snapshot,frontier.POSE,c)
        self.assertEqual(other['state'],'FRONTIERS_PRESENT')
        self.assertGreaterEqual(math.hypot(other['goal']['x_m']-goal['x_m'],other['goal']['y_m']-goal['y_m']),.20)
    def test_native_stream_defaults_remove_optional_python_rgb_preview(self):
        camera=camera_config({'camera':{'stream_backend':'web_video_server','stream_port':8080}})
        self.assertFalse(camera['mqtt_preview_enabled']);self.assertTrue(camera['stream_enabled'])

class MappingLateralTests(unittest.IsolatedAsyncioTestCase):
    async def test_sideways_only_navigation_keeps_lease_and_releases_it_on_stop(self):
        import test_autonomous_mapping as auto
        item,_=auto.MappingIntegrationTests().manager()
        item.auto.cfg['exploration_mode']='frontier';item.auto_selected=True
        item.auto.accept(dict(action='START',runId=auto.RUN,maxDurationSec=180,stopOnReturn=False),10.,auto.POSE,.9)
        item.adapter.cfg['motion']['allow_strafe']=True
        item.auto_scan=({**auto.CLEAR,'lateral_left_m':1.},10.)
        item.exploration=Mock();item.exploration.poll.return_value=({'state':'FRONTIERS_PRESENT'},None)
        item.exploration.no_progress.return_value=False
        item.navigator=N(active=True,tick=AsyncMock(return_value=((0.,.1,0.),'FOLLOW_FRONTIER_NAV2')),stop=AsyncMock())
        def twist():return N(linear=N(x=0.,y=0.,z=0.),angular=N(x=0.,y=0.,z=0.))
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=twist)}),patch('mediassist.mapping.time.monotonic',return_value=10.1):
            await item.tick(True,True)
            msg=item.velocity.publish.call_args.args[0]
            self.assertEqual(msg.linear.x,0.);self.assertGreater(msg.linear.y,0.)
            self.assertTrue(item.adapter.lease_message.call_args.args[0])
            item.auto.stop('OPERATOR_STOP');await item.tick(True,True)
            msg=item.velocity.publish.call_args.args[0]
            self.assertEqual((msg.linear.x,msg.linear.y,msg.angular.z),(0.,0.,0.))
            self.assertFalse(item.adapter.lease_message.call_args.args[0])

if __name__=='__main__':unittest.main()
