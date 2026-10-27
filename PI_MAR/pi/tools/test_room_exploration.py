"""Connected rooms, traversal memory, rear clearance and bounded lease loss. No motors/ROS."""
import copy,json,math,sys,unittest
from pathlib import Path
from unittest.mock import Mock,patch
ROOT=Path(__file__).resolve().parents[2];sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'pi/tools'),str(ROOT/'tools')]
from mediassist.frontier_mapping import plan
from mediassist.autonomous_mapping import AutoMapper,configuration,nav2_assist
from mediassist.contracts import StepError
from configure_room_exploration import tune
import test_frontier_mapping as frontier
import test_autonomous_mapping as auto

def profile():
    cfg=frontier.profile();cfg.update(frontier_controller='nav2',visit_known_space=True,visit_radius_m=.6,stop_margin_m=.05,
        footprint=dict(front=.12,rear=.12,left=.1,right=.1),reverse_enabled=True,max_reverse_mps=.1,control_pause_sec=2.)
    return configuration({'autonomy':cfg})

def rooms():
    w,h=120,90;data=bytearray([0]*(w*h))
    for y in range(h):
        for x in range(w):
            if x in (0,w-1) or y in (0,h-1) or x==60 and not 35<=y<=46 or y==65 and x>=60:data[y*w+x]=100
    return dict(width=w,height=h,resolution=.05,origin=[0.,0.,0.],frame_id='map',data=bytes(data))

class RoomTests(unittest.TestCase):
    def test_white_room_seen_from_corridor_is_not_coverage_complete(self):
        cfg=profile();snapshot=rooms();pose=dict(x_m=2.,y_m=2.,yaw_rad=0.)
        cfg['_visited_poses']=[(x/10,y/10) for x in range(4,28,3) for y in range(4,43,3)]
        result=plan(snapshot,pose,cfg)
        self.assertEqual(result['reason'],'VISIT_UNTRAVERSED_FREE_SPACE')
        walked=[]
        for _ in range(8):
            walked.extend(result['path']);cfg['_visited_poses'].extend(result['path']);pose=dict(result['goal'])
            if pose['x_m']>3.1:break
            result=plan(snapshot,pose,cfg);self.assertEqual(result['state'],'FRONTIERS_PRESENT')
        self.assertGreater(pose['x_m'],3.1)
        self.assertTrue(any(1.75<=y<=2.35 and 2.95<=x<=3.05 for x,y in walked))
        self.assertLess(result['goal']['y_m'],3.25) # upper sealed room cannot be reached

    def test_nearby_seen_room_is_visited_before_distant_frontier(self):
        cfg=profile();snapshot=rooms();raw=bytearray(snapshot['data'])
        for y in range(42,50):
            for x in range(2,6):raw[y*snapshot['width']+x]=255
        snapshot['data']=bytes(raw)
        pose=dict(x_m=1.,y_m=2.,yaw_rad=0.)
        cfg['_visited_poses']=[(x/10,y/10) for x in range(4,28,3) for y in range(4,43,3)]
        result=plan(snapshot,pose,cfg)
        self.assertGreater(result['frontier_count'],0)
        self.assertEqual(result['reason'],'VISIT_UNTRAVERSED_FREE_SPACE')
        self.assertGreater(result['goal']['x_m'],3.)
    def test_goal_completion_covers_connected_rooms_and_does_not_enter_sealed_room(self):
        cfg=profile();snapshot=rooms();pose=dict(x_m=1.,y_m=2.,yaw_rad=0.);visited=[(pose['x_m'],pose['y_m'])]
        for _ in range(100):
            result=plan(snapshot,pose,{**cfg,'_visited_poses':visited})
            if result['state']=='NO_FRONTIERS':break
            self.assertEqual(result['state'],'FRONTIERS_PRESENT')
            goal=result['goal'];self.assertFalse(goal['x_m']>3. and goal['y_m']>3.25)
            # Simulate a successful path traversal; this is not a physical Nav2 test.
            visited.extend(result['path']);pose=dict(goal)
        else:self.fail('Coverage did not converge on the finite two-room scene')
        self.assertTrue(any(x>3.5 for x,y in visited));self.assertEqual(result['reason'],'OBSERVED_AND_VISITED_REACHABLE_SPACE')
    def test_unknown_frontiers_have_priority_and_excluded_goals_cannot_mean_complete(self):
        cfg=profile();snapshot=frontier.room(open_side=True)
        result=plan(snapshot,frontier.POSE,cfg)
        self.assertEqual(result['reason'],'APPROACH_FRONTIER')
        snapshot=rooms();pose=dict(x_m=1.,y_m=2.,yaw_rad=0.)
        result=plan(snapshot,pose,{**cfg,'_excluded_goals':[(x*.1,y*.1) for x in range(61) for y in range(46)]})
        self.assertEqual(result['state'],'BLOCKED')
    def test_reverse_is_opt_in_uses_rear_corridor_and_respects_cap(self):
        cfg=profile();view=dict(front_m=.03,back_m=1.,lateral_left_m=1.,lateral_right_m=1.)
        velocity=nav2_assist((-.5,0.,0.),view,cfg,.1)
        self.assertGreaterEqual(velocity[0],-.1);self.assertLess(velocity[0],0.)
        self.assertEqual(nav2_assist((-.1,0.,0.),{**view,'back_m':.02},cfg)[0],0.)
        self.assertEqual(nav2_assist((-.1,0.,0.),view,{**cfg,'reverse_enabled':False})[0],0.)
        with self.assertRaises(StepError):configuration({'autonomy':{**cfg,'vehicle':'articulated'}})
    def test_sideways_nav2_command_ignores_negligible_reverse_drift(self):
        from mediassist.rear_scan import chassis_guard
        cfg=profile();view=dict(front_m=1.,back_m=1.,lateral_left_m=1.,lateral_right_m=1.)
        velocity=nav2_assist((-.001649,.14,-.216),view,cfg,.1)
        self.assertEqual(velocity[0],0.)
        motion=dict(chassis_model='MENTORPI_M1_MECANUM',allow_strafe=True,allow_reverse=True,
                    sensor_max_age_sec=.6,max_reverse_speed_mps=.1)
        self.assertEqual(chassis_guard(velocity,motion,'REAR_OBSTACLE',.1),(velocity,None))
        reverse=nav2_assist((-.05,0.,0.),view,cfg,.1)
        self.assertEqual(chassis_guard(reverse,motion,'REAR_OBSTACLE',.1)[1],'REAR_OBSTACLE')
        self.assertEqual(chassis_guard((-.0038,0.,-.05),motion,'REAR_OBSTACLE',.1),((0.,0.,-.05),None))

    def test_rear_block_holds_for_replan_then_stops_if_persistent(self):
        cfg=profile();mapper=AutoMapper(cfg);msg=dict(action='START',runId='test',maxDurationSec=100,stopOnReturn=False)
        pose=dict(x_m=0.,y_m=0.,yaw_rad=0.);mapper.accept(msg,10.,pose,.9)
        blocked=dict(reason='REAR_OBSTACLE')
        self.assertEqual(mapper.current(10.1,auto.CLEAR,0.,pose,True,gate=blocked),(0.,0.))
        self.assertEqual(mapper.phase,'RUNNING');self.assertEqual(mapper.reason,'REPLANNING_REAR_BLOCKED')
        mapper.accept({**msg,'action':'RENEW'},10.5,pose,.9)
        mapper.current(10.6,auto.CLEAR,0.,pose,True,gate={'reason':'CLEAR'},hold=True)
        self.assertIsNone(mapper.gate_blocked_at)
        mapper.current(10.7,auto.CLEAR,0.,pose,True,gate=blocked)
        for tick in range(11,20):mapper.accept({**msg,'action':'RENEW'},float(tick),pose,.9)
        mapper.current(19.1,auto.CLEAR,0.,pose,True,gate=blocked)
        self.assertEqual(mapper.phase,'STOPPED');self.assertEqual(mapper.reason,'GATE_REAR_OBSTACLE')
    def test_short_lease_gap_holds_zero_then_only_fresh_same_run_can_resume(self):
        cfg=profile();mapper=AutoMapper(cfg);msg=dict(action='START',runId='test',maxDurationSec=100,stopOnReturn=False)
        pose=dict(x_m=0.,y_m=0.,yaw_rad=0.);mapper.accept(msg,10.,pose,.9)
        velocity=mapper.current(11.,auto.CLEAR,0.,pose,True,hold=True)
        self.assertEqual(velocity,(0.,0.));self.assertEqual(mapper.phase,'RUNNING');self.assertEqual(mapper.reason,'WAITING_FOR_CONTROL_LEASE')
        mapper.accept({**msg,'action':'RENEW'},11.1,pose,.9);self.assertGreater(mapper.deadline,11.1)
        mapper.current(15.,auto.CLEAR,0.,pose,True,hold=True);self.assertEqual(mapper.phase,'STOPPED')
        with self.assertRaises(StepError):mapper.accept({**msg,'action':'RENEW'},15.1,pose,.9)
    def test_preset_preserves_camera_network_geometry_and_credentials(self):
        cfg={'mapping':{'autonomy':profile(),'camera':{'stream_backend':'web_video_server'},'upload_base_url':'https://be'},'mqtt':{'host':'broker'}}
        hw={'max_speed_mps':1.,'motion':dict(chassis_model='MENTORPI_M1_MECANUM',allow_strafe=True,allow_reverse=True,
            commissioned=True,cart_footprint_validated=True,watchdog_validated=True,max_reverse_speed_mps=.25,max_angular_speed_rps=.8)}
        before=copy.deepcopy(cfg);result=tune(cfg,hw)
        self.assertEqual(cfg,before);self.assertEqual(result['mqtt'],before['mqtt']);self.assertEqual(result['mapping']['camera'],before['mapping']['camera'])
        self.assertEqual(result['mapping']['autonomy']['footprint'],before['mapping']['autonomy']['footprint'])
        self.assertEqual(result['mapping']['autonomy']['exploration_mode'],'frontier')
        with self.assertRaises(ValueError):tune(cfg,{**hw,'motion':{**hw['motion'],'allow_reverse':False}})
    def test_mapping_yaw_is_optional_and_reverse_does_not_change_endpoint_navigation(self):
        from mapping_nav2_parameters import mapping_motion_parameters
        params={'controller_server':{'ros__parameters':{'FollowPath':{'min_vel_x':0.,'critics':['RotateToGoal','ObstacleFootprint']},
            'general_goal_checker':{'yaw_goal_tolerance':.1}}},'velocity_smoother':{'ros__parameters':{'min_velocity':[0.,-.25,-.6]}}}
        original=copy.deepcopy(params)
        hw={'motion':{'allow_reverse':True,'max_reverse_speed_mps':.25}}
        mapping_motion_parameters(params,profile(),hw)
        self.assertEqual(params['controller_server']['ros__parameters']['FollowPath']['min_vel_x'],-.1)
        self.assertEqual(params['velocity_smoother']['ros__parameters']['min_velocity'][0],-.1)
        self.assertEqual(params['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance'],math.pi)
        self.assertIn('ObstacleFootprint',params['controller_server']['ros__parameters']['FollowPath']['critics'])
        self.assertEqual(original['controller_server']['ros__parameters']['general_goal_checker']['yaw_goal_tolerance'],.1)

class LoopTests(unittest.IsolatedAsyncioTestCase):
    async def test_rear_gate_cancels_goal_and_holds_zero_for_new_plan(self):
        from types import SimpleNamespace as N
        from unittest.mock import AsyncMock
        item,_=auto.MappingIntegrationTests().manager()
        item.auto.cfg=profile();item.auto_selected=True
        item.auto.accept(dict(action='START',runId=auto.RUN,maxDurationSec=180,stopOnReturn=False),10.,auto.POSE,.9)
        item.grid=({},10.);item.auto_scan=({**auto.CLEAR,'back_m':1.},10.)
        item.adapter.telemetry=lambda:{'odom_speed':{'age_sec':.01},'map_pose':{'fresh':True,'value':auto.POSE},
                                       'gate':{'value':{'reason':'REAR_OBSTACLE'}}}
        goal=dict(x_m=1.,y_m=0.,yaw_rad=0.)
        item.exploration=Mock();item.exploration.poll.return_value=({'state':'FRONTIERS_PRESENT','goal':goal},None)
        item.navigator=N(active=True,target=goal,tick=AsyncMock(),stop=AsyncMock())
        def twist():return N(linear=N(x=0.,y=0.,z=0.),angular=N(x=0.,y=0.,z=0.))
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=twist)}),patch('mediassist.mapping.time.monotonic',return_value=10.1):
            await item.tick(True,True)
        item.navigator.stop.assert_awaited_once();item.navigator.tick.assert_not_awaited()
        item.exploration.exclude_goal.assert_called_once_with(goal,10.1,ttl=15.)
        self.assertEqual(item.auto.phase,'RUNNING');self.assertEqual(item.auto.reason,'REPLANNING_REAR_BLOCKED')
        msg=item.velocity.publish.call_args.args[0]
        self.assertEqual((msg.linear.x,msg.linear.y,msg.angular.z),(0.,0.,0.))

    async def test_expired_control_lease_cannot_publish_cached_nav2_velocity(self):
        from types import SimpleNamespace as N
        from unittest.mock import AsyncMock
        item,_=auto.MappingIntegrationTests().manager()
        item.auto.cfg=profile();item.auto_selected=True
        item.auto.accept(dict(action='START',runId=auto.RUN,maxDurationSec=180,stopOnReturn=False),10.,auto.POSE,.9)
        item.exploration=Mock();item.exploration.poll.return_value=({'state':'FRONTIERS_PRESENT','goal':dict(x_m=1.,y_m=0.,yaw_rad=0.)},None)
        item.navigator=N(active=True,tick=AsyncMock(return_value=((.25,.1,.3),'FOLLOW_FRONTIER_NAV2')),stop=AsyncMock())
        item.grid=({},11.);item.auto_scan=({**auto.CLEAR,'back_m':1.},11.)
        def twist():return N(linear=N(x=0.,y=0.,z=0.),angular=N(x=0.,y=0.,z=0.))
        with patch.dict(sys.modules,{'geometry_msgs.msg':N(Twist=twist)}),patch('mediassist.mapping.time.monotonic',return_value=11.):
            await item.tick(True,True)
        self.assertEqual(item.auto.phase,'RUNNING');self.assertEqual(item.auto.reason,'WAITING_FOR_CONTROL_LEASE')
        msg=item.velocity.publish.call_args.args[0]
        self.assertEqual((msg.linear.x,msg.linear.y,msg.angular.z),(0.,0.,0.))
        self.assertFalse(item.adapter.lease_message.call_args.args[0]);item.navigator.tick.assert_not_awaited()

if __name__=='__main__':unittest.main()
