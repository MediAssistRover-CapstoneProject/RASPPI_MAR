"""Fast mapping regression without ROS, GPIO, network or physical motion."""
import copy
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as N
import unittest
from unittest.mock import AsyncMock, Mock, patch
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools'),str(ROOT/'pi/tools')]
from configure_autonomy_speed import tune
from mediassist.autonomous_mapping import configuration, nav2_assist
from mediassist.configuration import validate_config
from mediassist.contracts import StepError
from mediassist.navigation_setup import nav2_params
from mediassist.safety import gate_velocity
import test_autonomous_mapping as fixtures


def inputs():
    hardware=json.loads((ROOT/'pi/config/site.example.json').read_text())
    hardware['max_speed_mps']=.25
    hardware['motion'].update(chassis_model='MENTORPI_M1_MECANUM',allow_strafe=True,
        commissioned=True,cart_footprint_validated=True,watchdog_validated=True,max_angular_speed_rps=.8)
    auto=fixtures.settings()
    auto.update(vehicle='rigid',exploration_mode='frontier',frontier_controller='nav2',speed_mps=.18,
                footprint=dict(front=.12,rear=.12,left=.1,right=.1),slow_clearance_m=.45)
    cfg={'hardware_profile':'existing.json','mqtt':{'host':'unchanged'},'mapping':{'autonomy':auto,
         'camera':{'stream_backend':'web_video_server'},'gamepad':{'forward_mps':.25},
         'upload_base_url':'https://unchanged','upload_token_file':'private'}}
    return cfg,hardware


class SpeedTests(unittest.TestCase):
    def test_requested_one_metre_speed_and_reactive_alternative(self):
        cfg,hw=tune(*inputs(),1.,.8)
        auto=cfg['mapping']['autonomy']
        self.assertEqual((auto['speed_mps'],auto['angular_rps']),(1.,.8))
        self.assertGreaterEqual(auto['slow_clearance_m'],1.9)
        self.assertEqual(hw['max_speed_mps'],1.)
        view=dict(front_m=3.,lateral_left_m=3.,lateral_right_m=3.)
        x,y,z=nav2_assist((1.,1.,.8),view,auto,.1)
        self.assertAlmostEqual(math.hypot(x,y),1.);self.assertEqual(z,.8)
        near={**view,'front_m':.8}
        fresh=nav2_assist((1.,0.,0.),near,auto,.01)[0]
        older=nav2_assist((1.,0.,0.),near,auto,.39)[0]
        self.assertLessEqual(older,fresh)
        self.assertLessEqual(older*(.39+.35)+older**2/(2*.6),.8-auto['stop_margin_m'])
        reactive,_=tune(cfg,hw,.2,.8,mode='reactive')
        self.assertEqual(reactive['mapping']['autonomy']['exploration_mode'],'reactive')
        fast_reactive,_=tune(cfg,hw,1.,.8,mode='reactive')
        self.assertEqual(fast_reactive['mapping']['autonomy']['stop_margin_m'],.05)

    def test_only_mapping_speed_and_hardware_ceiling_change(self):
        cfg,hw=inputs();before=copy.deepcopy(cfg);before_hw=copy.deepcopy(hw)
        result,tuned=tune(cfg,hw,.5)
        self.assertEqual(cfg,before);self.assertEqual(hw,before_hw)
        for key in ('camera','gamepad','upload_base_url','upload_token_file'):
            self.assertEqual(result['mapping'][key],cfg['mapping'][key])
        self.assertEqual(tuned['motion'],hw['motion'])
        self.assertEqual(tuned['max_speed_mps'],.5)
        self.assertEqual(result['mapping']['autonomy']['speed_mps'],.5)
        self.assertEqual(result['mapping']['autonomy']['slow_clearance_m'],.8)
        self.assertEqual(result['mapping']['autonomy']['scan_max_age_sec'],.4)
        self.assertEqual(result['mapping']['autonomy']['footprint'],cfg['mapping']['autonomy']['footprint'])
        slower,_=tune(result,tuned,.18)
        self.assertEqual(slower['mapping']['autonomy']['speed_mps'],.18)

    def test_invalid_speed_and_uncommissioned_or_trailer_profiles_rejected(self):
        for speed in (1.001,float('nan'),float('inf'),True,0.):
            with self.subTest(speed=speed),self.assertRaises(ValueError):tune(*inputs(),speed)
        cfg,hw=inputs();hw['motion']['commissioned']=False
        with self.assertRaises(ValueError):tune(cfg,hw,.5)
        cfg,hw=inputs();cfg['mapping']['autonomy']['vehicle']='articulated'
        with self.assertRaises(ValueError):tune(cfg,hw,.5)
        cfg,hw=inputs();hw['motion']['chassis_model']='OTHER';hw['max_speed_mps']=.5
        with self.assertRaises(StepError):validate_config(hw)

    def test_direct_json_edit_cannot_skip_fast_mode_sensor_policy(self):
        cfg,hw=inputs();result,_=tune(cfg,hw,.5)
        for change in ({'slow_clearance_m':.45},{'scan_max_age_sec':.61},{'vehicle':'articulated'},
                       {'frontier_controller':'reactive'},{'speed_mps':1.001}):
            auto={**result['mapping']['autonomy'],**change}
            with self.subTest(change=change),self.assertRaises(StepError):configuration({'autonomy':auto})

    def test_nav2_high_speed_requires_explicit_mapping_and_mecanum(self):
        base={'amcl':{'ros__parameters':{}},'velocity_smoother':{'ros__parameters':{}},
              'planner_server':{'ros__parameters':{'GridBased':{}}}}
        for name in ('local_costmap','global_costmap'):
            base[name]={name:{'ros__parameters':{'inflation_layer':{}}}}
        ctl={'controller_server':{'ros__parameters':{'FollowPath':{'plugin':'dwb_core::DWBLocalPlanner','critics':['BaseObstacle']},'progress_checker':{},'general_goal_checker':{}}}}
        for kwargs in ({},{'mecanum':True},{'fast_mapping':True}):
            with self.assertRaises(ValueError):nav2_params(base,ctl,[],.5,.6,**kwargs)
        out=nav2_params(base,ctl,[],.5,.6,mecanum=True,fast_mapping=True)
        self.assertEqual(out['controller_server']['ros__parameters']['FollowPath']['max_speed_xy'],.5)
        self.assertEqual(out['velocity_smoother']['ros__parameters']['max_velocity'],[.5,.5,.6])

    def test_obstacle_slowdown_and_gate_keep_vector_cap(self):
        cfg,hw=tune(*inputs(),.5);auto=cfg['mapping']['autonomy']
        view=dict(front_m=2.,lateral_left_m=2.,lateral_right_m=2.)
        velocity=nav2_assist((.5,.5,0),view,auto)
        self.assertAlmostEqual(math.hypot(*velocity[:2]),.5)
        self.assertLess(nav2_assist((.5,0,0),{**view,'front_m':.45},auto)[0],.3)
        self.assertEqual(nav2_assist((.5,0,0),{**view,'front_m':.05},auto),(0.,0.,0.))
        hw['motion']['us100_required']=False
        applied,reason=gate_velocity(velocity,[],10,10,10,10,hw['motion'],.5)
        self.assertEqual(reason,'CLEAR');self.assertAlmostEqual(math.hypot(*applied[:2]),.5)


class SpeedIntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def test_mapping_reaches_half_metre_vector_and_still_stops_on_stale_scan(self):
        item,_=fixtures.MappingIntegrationTests().manager()
        cfg,hw=tune(*inputs(),.5)
        item.auto.cfg=cfg['mapping']['autonomy'];item.auto_selected=True
        item.adapter.cfg=hw
        item.auto.accept(dict(action='START',runId=fixtures.RUN,maxDurationSec=180,stopOnReturn=False),10.,fixtures.POSE,.9)
        item.exploration=Mock();item.exploration.poll.return_value=({'state':'FRONTIERS_PRESENT'},None)
        item.exploration.no_progress.return_value=False
        item.navigator=N(active=True,tick=AsyncMock(return_value=((.5,.5,0.),'FOLLOW_FRONTIER_NAV2')),stop=AsyncMock())
        def twist():return N(linear=N(x=0.,y=0.,z=0.),angular=N(x=0.,y=0.,z=0.))
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=twist)}),patch('mediassist.mapping.time.monotonic') as clock:
            for i in range(32):
                now=10.1+i*.05;clock.return_value=now
                item.grid=({},now);item.auto_scan=({**fixtures.CLEAR,'lateral_left_m':3.},now)
                item.auto.accept(dict(action='RENEW',runId=fixtures.RUN,maxDurationSec=180,stopOnReturn=False),now,fixtures.POSE,.9)
                await item.tick(True,True)
            msg=item.velocity.publish.call_args.args[0]
            self.assertAlmostEqual(math.hypot(msg.linear.x,msg.linear.y),.5)
            self.assertEqual(item.adapter.lease_message.call_args.args,(True,.5))
            clock.return_value=now+.41
            await item.tick(True,True)
            msg=item.velocity.publish.call_args.args[0]
            self.assertEqual((msg.linear.x,msg.linear.y,msg.angular.z),(0.,0.,0.))
            self.assertFalse(item.adapter.lease_message.call_args.args[0])


if __name__=='__main__':unittest.main()
