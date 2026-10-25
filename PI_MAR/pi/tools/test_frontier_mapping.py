"""Synthetic rooms, planner lifecycle and real mapping-loop stop publication; no hardware."""
import asyncio
import copy
import json
import math
from pathlib import Path
import sys
import unittest
import tempfile
from types import SimpleNamespace as NS
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'pi/tools')]
from mediassist.frontier_mapping import plan,Tracker,FrontierWorker,guidance,source_signature,squared_distances,visible
from mediassist.autonomous_mapping import AutoMapper,manual_assist
from jsonschema import Draft202012Validator
import test_autonomous_mapping as fixtures


def room(open_side=False):
    w=h=50;data=bytearray([255]*(w*h))
    for y in range(3,47):
        for x in range(3,47):data[y*w+x]=100 if x in (3,46) or y in (3,46) else 0
    if open_side:
        for y in range(18,32):
            for x in range(35,47):data[y*w+x]=255
    return dict(width=w,height=h,resolution=.1,origin=[0.,0.,0.],frame_id='map',data=bytes(data))


def profile():
    c=fixtures.settings();c['exploration_mode']='frontier';c['frontier_controller']='reactive';return c


POSE=dict(x_m=2.05,y_m=2.55,yaw_rad=0.)


class FrontierTests(unittest.TestCase):
    def test_euclidean_field_matches_brute_force_distances_including_boundaries(self):
        import random
        generator=random.Random(42)
        for w,h in ((1,1),(3,7),(11,9)):
            cells=bytearray(1 if generator.random()<.15 else 0 for _ in range(w*h))
            actual=squared_distances(cells,w,h)
            obstacles=[(i%w,i//w) for i in range(w*h) if cells[i] or i%w in (0,w-1) or i//w in (0,h-1)]
            for i,value in enumerate(actual):
                self.assertEqual(value,min((i%w-x)**2+(i//w-y)**2 for x,y in obstacles))

    def fine_corridor(self):
        w,h=160,60;raw=bytearray([255]*(w*h))
        for y in range(10,39):
            for x in range(4,101):raw[y*w+x]=100 if x==4 or y in (10,38) else 0
        # Open the right end into unknown space; side walls and black cells remain real obstacles.
        for y in range(11,38):raw[y*w+100]=255
        return dict(width=w,height=h,resolution=.025,origin=[0.,0.,0.],frame_id='map',data=bytes(raw))

    def test_native_resolution_keeps_a_clear_seventy_centimetre_corridor_traversable(self):
        cfg=profile();cfg['footprint']=dict(front=.12,rear=.12,left=.10,right=.10)
        snapshot=self.fine_corridor();before=snapshot['data']
        result=plan(snapshot,dict(x_m=1.0125,y_m=.6125,yaw_rad=0.),cfg)
        self.assertEqual(result['state'],'FRONTIERS_PRESENT')
        self.assertEqual(result['planner_cell_m'],.025)
        self.assertGreater(result['seed_clearance_m'],result['clearance_radius_m'])
        self.assertEqual(snapshot['data'],before)

    def test_obstacle_buffer_and_unknown_body_overlap_still_block(self):
        cfg=profile();cfg['footprint']=dict(front=.12,rear=.12,left=.10,right=.10)
        for obstacle in (100,255):
            snapshot=self.fine_corridor();raw=bytearray(snapshot['data']);raw[24*160+47]=obstacle;snapshot['data']=bytes(raw)
            result=plan(snapshot,dict(x_m=1.0125,y_m=.6125,yaw_rad=0.),cfg)
            self.assertEqual(result['state'],'BLOCKED')
            self.assertEqual(result['reason'],'ROBOT_FOOTPRINT_NOT_CLEAR_IN_MAP')

    def test_unknown_outside_the_full_body_does_not_receive_an_obstacle_buffer(self):
        cfg=profile();cfg['footprint']=dict(front=.12,rear=.12,left=.10,right=.10)
        for value in (255,100):
            snapshot=self.fine_corridor();raw=bytearray(snapshot['data']);raw[24*160+49]=value;snapshot['data']=bytes(raw)
            result=plan(snapshot,dict(x_m=1.0125,y_m=.6125,yaw_rad=0.),cfg)
            if value==255:
                self.assertGreater(result['seed_unknown_clearance_m'],result['body_radius_m'])
                self.assertNotEqual(result['reason'],'ROBOT_FOOTPRINT_NOT_CLEAR_IN_MAP')
                # The unknown cell can still cut the narrow route to the distant frontier.
                self.assertEqual(result['state'],'BLOCKED')
            else:self.assertEqual(result['reason'],'ROBOT_FOOTPRINT_NOT_CLEAR_IN_MAP')

    def test_nearby_goal_does_not_hold_before_robot_has_reached_it(self):
        result=dict(state='FRONTIERS_PRESENT',goal=dict(x_m=.08,y_m=0.,yaw_rad=0.),
                    path=[(0.,0.),(.08,0.)],planner_cell_m=.025)
        self.assertFalse(guidance(result,dict(x_m=0.,y_m=0.,yaw_rad=0.))['hold'])
        self.assertTrue(guidance(result,dict(x_m=.07,y_m=0.,yaw_rad=0.))['hold'])

    def test_lookahead_cannot_cut_across_a_blocked_corner(self):
        safe=bytearray([1]*100);safe[5*10+3]=0
        grid=dict(width=10,height=10,cell=.1,origin=[0.,0.,0.],safe=bytes(safe))
        self.assertFalse(visible(grid,(.25,.45),(.45,.65)))
        self.assertTrue(visible(grid,(.25,.45),(.25,.65)))
        result=dict(state='FRONTIERS_PRESENT',goal=dict(x_m=.55,y_m=.65,yaw_rad=0.),planner_cell_m=.1,
            planning_grid=grid,path=[(.25,.45),(.25,.55),(.25,.65),(.35,.65),(.45,.65),(.55,.65)])
        heading=guidance(result,dict(x_m=.25,y_m=.45,yaw_rad=0.))
        self.assertAlmostEqual(heading['yaw'],math.pi/2)

    def test_visibility_handles_rotated_map_origins(self):
        safe=bytearray([1]*100);safe[5*10+3]=0
        grid=dict(width=10,height=10,cell=.1,origin=[10.,20.,math.pi/2],safe=bytes(safe))
        self.assertFalse(visible(grid,(10.-.45,20.+.25),(10.-.65,20.+.45)))
        self.assertTrue(visible(grid,(10.-.45,20.+.25),(10.-.65,20.+.25)))

    def test_profile_plan_keeps_motion_flags_and_allows_bounded_slow_trailer_turn(self):
        sys.path.insert(0,str(ROOT/'tools'))
        import configure_autonomous_mapping as tool
        import test_camera_config as config_fixtures
        with tempfile.TemporaryDirectory() as folder:
            config,_,hardware=config_fixtures.CameraConfigTests().fixture(folder)
            hw=json.loads(hardware.read_bytes());hw['max_speed_mps']=.15;hw['motion']['max_angular_speed_rps']=.5
            hardware.write_text(json.dumps(hw));before=config.read_bytes()
            from unittest.mock import patch
            with patch('builtins.print'):
                cfg=tool.configure(config,True,dict(front=.2,rear=.4,left=.3,right=.3),speed=.06,angular=.15,
                                   duration=300,vehicle='articulated',turn_radius=.6,exploration='frontier',manual_assist=True)
            self.assertEqual(config.read_bytes(),before)
            self.assertEqual(cfg['mapping']['autonomy']['blocked_timeout_sec'],20.)
            self.assertTrue(cfg['mapping']['autonomy']['manual_lidar_assist'])
            self.assertEqual(json.loads(hardware.read_bytes())['motion'],hw['motion'])

    def test_closed_room_with_black_walls_and_gray_exterior_has_no_frontier(self):
        r=plan(room(),POSE,profile())
        self.assertEqual(r['state'],'NO_FRONTIERS');self.assertEqual(r['frontier_count'],0)
        self.assertGreater(r['reachable_free_area_m2'],10.)

    def test_no_frontiers_cannot_complete_when_occupied_map_overlaps_body(self):
        snapshot=room();raw=bytearray(snapshot['data']);raw[25*50+21]=100
        snapshot['data']=bytes(raw)
        result=plan(snapshot,POSE,profile())
        self.assertEqual(result['frontier_count'],0)
        self.assertEqual(result['state'],'BLOCKED')
        self.assertEqual(result['reason'],'ROBOT_FOOTPRINT_NOT_CLEAR_IN_MAP')
        tracker=Tracker(profile())
        for now,revision in ((40.,1),(50.,2),(60.,3)):tracker.observe(result,now,revision,now,5.)
        self.assertFalse(tracker.complete)

    def test_unknown_inside_room_is_a_real_frontier_not_completion(self):
        r=plan(room(True),POSE,profile())
        self.assertEqual(r['state'],'FRONTIERS_PRESENT');self.assertGreater(r['frontier_count'],0)
        self.assertTrue(r['path']);self.assertGreater(r['goal']['x_m'],POSE['x_m'])
        self.assertIsNotNone(guidance(r,POSE))

    def test_uncertain_probability_band_cannot_fake_an_enclosing_wall(self):
        s=room();data=bytearray(s['data'])
        for y in range(4,46):data[y*50+46]=50
        s['data']=bytes(data)
        self.assertNotEqual(plan(s,POSE,profile())['state'],'NO_FRONTIERS')

    def test_free_cells_at_map_edge_are_not_a_closed_room(self):
        s=room();s['data']=bytes([0]*2500)
        self.assertNotEqual(plan(s,POSE,profile())['state'],'NO_FRONTIERS')

    def test_unknown_behind_a_closed_wall_is_excluded(self):
        s=room();data=bytearray(s['data'])
        for y in range(4,46):data[y*50+25]=100
        for y in range(4,46):
            for x in range(26,46):data[y*50+x]=255
        s['data']=bytes(data)
        self.assertEqual(plan(s,POSE,profile())['state'],'NO_FRONTIERS')

    def test_narrow_passage_or_insufficient_body_clearance_is_blocked_not_complete(self):
        cfg=profile();cfg['footprint']=dict(front=1.,rear=1.,left=1.,right=1.)
        r=plan(room(True),dict(x_m=.55,y_m=.55,yaw_rad=0.),cfg)
        self.assertEqual(r['state'],'BLOCKED');self.assertGreater(r['frontier_count'],0)

    def test_target_path_stays_on_known_free_cells_and_never_crosses_wall(self):
        s=room(True);data=bytearray(s['data'])
        for y in range(4,36):data[y*50+27]=100
        s['data']=bytes(data);r=plan(s,POSE,profile())
        self.assertEqual(r['state'],'FRONTIERS_PRESENT')
        for x,y in r['path']:self.assertEqual(data[int(y/.1)*50+int(x/.1)],0)
        self.assertTrue(any(y>3.6 for x,y in r['path']))

    def test_rotated_origin_and_outside_pose(self):
        s=room(True);s['origin']=[10.,20.,math.pi/2]
        p=dict(x_m=10.-POSE['y_m'],y_m=20.+POSE['x_m'],yaw_rad=math.pi/2)
        r=plan(s,p,profile());self.assertEqual(r['state'],'FRONTIERS_PRESENT')
        self.assertAlmostEqual(r['goal']['yaw_rad'],math.pi/2,delta=.3)
        self.assertEqual(plan(s,POSE,profile())['state'],'BLOCKED')

    def test_complete_needs_stable_content_distinct_maps_time_and_travel(self):
        cfg=profile();cfg['coverage_min_travel_m']=.5;t=Tracker(cfg);r=plan(room(),POSE,cfg)
        for now,rev in [(0.,1),(8.,2),(16.,3)]:t.observe(r,now,rev,40.,1.)
        self.assertTrue(t.complete);self.assertEqual(t.status()['state'],'COMPLETE')
        for condition in ['revision','travel','duration']:
            t=Tracker(cfg)
            for now,rev in [(0.,1),(8.,2),(16.,3)]:
                t.observe(r,now,1 if condition=='revision' else rev,5. if condition=='duration' else 40.,0. if condition=='travel' else 1.)
            self.assertFalse(t.complete)

    def test_new_frontier_or_map_change_resets_confirmation(self):
        cfg=profile();t=Tracker(cfg);closed=plan(room(),POSE,cfg)
        t.observe(closed,0.,1,40.,1.);t.observe(closed,10.,2,50.,1.)
        t.observe(plan(room(True),POSE,cfg),16.,3,56.,1.)
        self.assertFalse(t.complete)
        t.observe(closed,17.,4,57.,1.);self.assertFalse(t.complete)

    def test_manual_assist_slows_stops_and_never_steers_without_input(self):
        c=profile();clear=fixtures.CLEAR
        (x,z),_=manual_assist((.1,0.),{**clear,'front_m':.3},.01,c)
        self.assertGreater(x,0);self.assertLess(x,.1);self.assertEqual(z,0)
        self.assertEqual(manual_assist((.1,0.),{**clear,'front_m':.05},.01,c)[0],(0.,0.))
        self.assertEqual(manual_assist((.1,0.),clear,2.,c)[0],(0.,0.))
        c['vehicle']='articulated'
        self.assertEqual(manual_assist((0.,.3),clear,.01,c)[0],(0.,0.))

    def test_guidance_keeps_speed_and_curvature_limits_and_stale_sensor_stop(self):
        auto=AutoMapper(profile());message=dict(action='START',runId=fixtures.RUN,maxDurationSec=180,stopOnReturn=False)
        auto.accept(message,0.,POSE,.9)
        x,z=auto.current(.1,fixtures.CLEAR,.01,POSE,True,guidance={'yaw':.1,'hold':False})
        self.assertGreater(x,0);self.assertLessEqual(x,auto.cfg['speed_mps']);self.assertGreater(z,0)
        self.assertEqual(auto.current(.2,fixtures.CLEAR,1.,POSE,True,guidance={'yaw':0.,'hold':False}),(0.,0.))
        self.assertEqual(auto.reason,'LIDAR_STALE')

    def test_trailer_turns_forward_with_bounded_radius_and_observes_without_pivot(self):
        cfg=profile();cfg['vehicle']='articulated';auto=AutoMapper(cfg)
        auto.accept(dict(action='START',runId=fixtures.RUN,maxDurationSec=180,stopOnReturn=False),0.,POSE,.9)
        view={**fixtures.CLEAR,'arc_left_clear':True,'arc_right_clear':True}
        x,z=auto.current(.1,view,.01,POSE,True,guidance={'yaw':math.pi/2,'hold':False})
        self.assertGreater(x,0);self.assertGreater(z,0);self.assertGreaterEqual(x/z,cfg['min_turn_radius_m'])
        self.assertEqual(auto.current(.2,view,.01,POSE,True,guidance={'yaw':math.pi/2,'hold':True}),(0.,0.))
        self.assertEqual(auto.phase,'RUNNING')

    def test_new_obstacle_stops_motion_even_with_a_previous_clear_frontier_path(self):
        auto=AutoMapper(profile())
        auto.accept(dict(action='START',runId=fixtures.RUN,maxDurationSec=180,stopOnReturn=False),0.,POSE,.9)
        velocity=auto.current(.1,{**fixtures.CLEAR,'front_m':.05},.01,POSE,True,guidance={'yaw':0.,'hold':False})
        self.assertEqual(velocity,(0.,0.));self.assertEqual(auto.reason,'FRONT_OBSTACLE_TOO_CLOSE')
        self.assertFalse(auto.status(.1)['coverage_complete'])

    def test_completed_status_contract_requires_real_completion_evidence(self):
        schema=json.loads((ROOT/'handoff/mapping-autonomy-status.schema.json').read_text())
        v=Draft202012Validator(schema)
        status=dict(state='STOPPED',finished=True,reason='COVERAGE_COMPLETE',coverage_complete=True,exploration_mode='frontier',
                    exploration=dict(enabled=True,state='COMPLETE',frontier_count=0))
        self.assertTrue(v.is_valid(status))
        for change in [{'reason':'TIME_LIMIT'},{'exploration':{'enabled':True,'state':'BLOCKED','frontier_count':1}},{'state':'RUNNING'}]:
            self.assertFalse(v.is_valid({**status,**change}))

    def test_virtual_room_exploration_advances_frontiers_then_confirms_completion(self):
        cfg=profile();auto=AutoMapper(cfg);tracker=Tracker(cfg)
        pose=dict(x_m=1.05,y_m=2.05,yaw_rad=0.)
        message=dict(action='START',runId=fixtures.RUN,maxDurationSec=180,stopOnReturn=False)
        auto.accept(message,0.,pose,.9)
        w,h=80,40;known=22
        for step in range(1,260):
            now=step*.5
            known=max(known,min(77,int(pose['x_m']/.1)+16))
            data=bytearray([255]*(w*h))
            for y in range(3,37):
                for x in range(3,known+1):data[y*w+x]=100 if x in (3,77) or y in (3,36) else 0
            snap=dict(width=w,height=h,resolution=.1,origin=[0.,0.,0.],frame_id='map',data=bytes(data))
            result=plan(snap,pose,cfg);self.assertNotEqual(result['state'],'BLOCKED')
            tracker.observe(result,now,step,now,auto.travel)
            auto.accept({**message,'action':'RENEW'},now,pose,.9)
            x,z=auto.current(now,fixtures.CLEAR,.01,pose,True,guidance=guidance(result,pose),hold=result['state']=='NO_FRONTIERS')
            self.assertAlmostEqual(z,0.,places=9,msg='Straight virtual room needs no turn')
            pose['x_m']+=x*.5
            if tracker.complete:break
        self.assertTrue(tracker.complete)
        self.assertGreater(auto.travel,3.)
        self.assertEqual(result['frontier_count'],0)


class AsyncTests(unittest.IsolatedAsyncioTestCase):
    async def test_rejected_goal_during_pending_plan_replans_without_false_timeout(self):
        worker=FrontierWorker(profile());worker.run=fixtures.RUN
        pending=asyncio.get_running_loop().create_future();worker.task=pending;worker.submitted_at=10.
        worker.exclude_goal(dict(x_m=1.,y_m=1.),10.1)
        result,error=worker.poll(room(),POSE,1,10.2,fixtures.RUN,30.,1.)
        self.assertIsNone(error);self.assertIsNone(result);self.assertEqual(worker.submitted_at,10.)
        pending.set_result(plan(room(True),POSE,profile()))
        result,error=worker.poll(room(),POSE,2,10.3,fixtures.RUN,30.,1.)
        self.assertIsNone(error);self.assertIsNone(result);self.assertIsNot(worker.task,pending)
        await worker.task

    async def test_no_progress_timer_starts_at_goal_and_resets_on_new_information(self):
        worker=FrontierWorker(profile());worker.tracker.progress_signature='one'
        self.assertFalse(worker.no_progress(100.,True))
        self.assertFalse(worker.no_progress(110.,True))
        self.assertTrue(worker.no_progress(116.,True))
        worker.tracker.progress_signature='two'
        self.assertFalse(worker.no_progress(117.,True))
        self.assertFalse(worker.no_progress(130.,False))

    async def test_planner_starts_in_background_then_returns_and_resets_run(self):
        worker=FrontierWorker(profile())
        r,error=worker.poll(room(True),POSE,1,0.,fixtures.RUN,0.,0.)
        self.assertIsNone(r);self.assertIsNone(error)
        await worker.task
        r,error=worker.poll(room(True),POSE,2,.2,fixtures.RUN,.2,0.)
        self.assertEqual(r['state'],'FRONTIERS_PRESENT')
        worker.reset('new-run');self.assertIsNone(worker.tracker.result)

    async def test_slow_planner_stops_without_blocking_control_loop(self):
        worker=FrontierWorker(profile());pending=asyncio.get_running_loop().create_future()
        worker.run=fixtures.RUN;worker.task=pending;worker.submitted_at=0.
        r,error=worker.poll(room(),POSE,1,3.1,fixtures.RUN,30.,1.)
        self.assertEqual(error,'EXPLORATION_PLAN_TIMEOUT');pending.cancel()

    async def test_mapping_loop_publishes_zero_and_complete_flag_only_for_fresh_confirmed_map(self):
        fixture=fixtures.MappingIntegrationTests();item,_=fixture.manager()
        item.revision=10;item.grid=(room(),10.);item.auto=AutoMapper(profile())
        item.auto.accept(dict(action='START',runId=fixtures.RUN,maxDurationSec=180,stopOnReturn=False),0.,POSE,.9)
        item.auto.deadline=11.;item.auto.travel=1.;item.auto_selected=True
        worker=FrontierWorker(item.auto.cfg);worker.run=fixtures.RUN
        result=plan(room(),POSE,item.auto.cfg)
        for now,rev in [(0.,1),(8.,2),(16.,3)]:worker.tracker.observe(result,now,rev,40.,1.)
        item.exploration=worker;worker.observed_at=10.;worker.submitted_at=10.
        item.adapter.telemetry=lambda:{'odom_speed':{'age_sec':.01},'map_pose':{'fresh':True,'value':POSE},'gate':{'value':{'reason':'CLEAR'}}}
        def twist():return NS(linear=NS(x=0.,y=0.,z=0.),angular=NS(x=0.,y=0.,z=0.))
        with patch('mediassist.mapping.time.monotonic',return_value=10.),patch.dict(sys.modules,{'geometry_msgs.msg':NS(Twist=twist)}):
            await item.tick(True,True)
        self.assertEqual(item.auto.reason,'COVERAGE_COMPLETE')
        self.assertTrue(item.state['autonomy']['coverage_complete'])
        self.assertEqual(item.velocity.publish.call_args.args[0].linear.x,0.)
        self.assertFalse(item.adapter.lease_message.call_args.args[0])


if __name__=='__main__':unittest.main()
