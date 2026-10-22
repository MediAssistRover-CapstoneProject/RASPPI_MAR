"""Fresh SLAM graphs, stale ROS frames and owned process restarts; no hardware or ROS needed."""
import asyncio
from array import array
import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import AsyncMock,Mock,patch

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'pi'),str(ROOT/'tools')]
from mediassist.slam_session import Engine,engine_parameters
from mediassist.mapping import MappingManager
from mediassist.contracts import StepError


class SlamTests(unittest.IsolatedAsyncioTestCase):
    def test_parameters_keep_vendor_solver_and_sensor_frames_but_never_continue_old_graph(self):
        source={'/**':{'ros__parameters':{'solver_plugin':'solver_plugins::CeresSolver','use_scan_matching':True,
            'map_file_name':'old-room','map_start_pose':[0,0,0],'map_start_at_dock':True,'mode':'localization'}}}
        before=copy.deepcopy(source)
        mapping={'slam_resolution_m':.025,'slam_min_travel_m':.05,'slam_min_heading_rad':.1,
                 'slam_min_interval_sec':.15,'slam_map_interval_sec':.5}
        ros={'map_frame':'map','odom_frame':'odom','base_frame':'base_footprint','scan':'/scan_raw'}
        result=engine_parameters(source,mapping,ros)['slam_toolbox']['ros__parameters']
        self.assertEqual(source,before)
        for key in ('map_file_name','map_start_pose','map_start_at_dock'):self.assertNotIn(key,result)
        self.assertEqual(result['solver_plugin'],'solver_plugins::CeresSolver');self.assertTrue(result['use_scan_matching'])
        self.assertEqual(result['resolution'],.025);self.assertEqual(result['minimum_travel_heading'],.1)
        self.assertEqual(result['scan_topic'],'/scan_raw');self.assertEqual(result['mode'],'mapping')
        self.assertFalse(result['use_sim_time'])

    def test_engine_restart_only_signals_verified_owned_child_and_changes_generation(self):
        with tempfile.TemporaryDirectory() as folder:
            engine=Engine(Path(folder)/'params.yaml',Path(folder)/'engine.log')
            first=Mock(pid=1200);second=Mock(pid=1300);first.poll.return_value=None;second.poll.return_value=None
            identities=[{'pid':1200,'ticks':'10'},{'pid':1300,'ticks':'11'}]
            inspect=Mock(side_effect=identities);terminate=Mock()
            with patch('mediassist.slam_session.subprocess.Popen',side_effect=[first,second]) as spawn,patch('mediassist.slam_session.process_tools',return_value=(inspect,terminate)):
                generation=engine.start();next_generation=engine.restart()
                self.assertNotEqual(generation,next_generation)
                terminate.assert_called_once_with(identities[0],grace=4.)
                self.assertTrue(spawn.call_args.kwargs['start_new_session'])
                self.assertIn('sync_slam_toolbox_node',spawn.call_args.args[0])
                engine.stop();self.assertEqual(terminate.call_args.args[0],identities[1])

    def manager(self):
        manager=object.__new__(MappingManager)
        manager.lock=threading.Lock();manager.accept_after_ns=10_000_000_000
        manager.cfg={'max_cells':100}
        manager.adapter=NS(cfg={'ros':{'map_frame':'map'}},node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)),get_logger=lambda:Mock()))
        manager.grid=None;manager.revision=0
        return manager

    @staticmethod
    def grid(sec):
        return NS(header=NS(frame_id='map',stamp=NS(sec=sec,nanosec=0)),
            info=NS(width=2,height=2,resolution=.025,origin=NS(position=NS(x=-1.,y=-2.),orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=array('b',[0,-1,100,0]))

    def test_old_ros_map_from_previous_graph_cannot_enter_new_session(self):
        manager=self.manager();manager.on_map(self.grid(9))
        self.assertIsNone(manager.grid);self.assertEqual(manager.revision,0)
        manager.on_map(self.grid(11));self.assertEqual(manager.revision,1)
        snapshot=manager.grid[0];self.assertIn('1970-01-01T00:00:11',snapshot['captured_at'])
        self.assertEqual(snapshot['origin'],[-1.,-2.,0.])

    def test_reset_barrier_rechecks_after_callback_waits_for_lock(self):
        manager=self.manager();msg=self.grid(11)
        class Barrier:
            def __enter__(self):manager.accept_after_ns=12_000_000_000
            def __exit__(self,*_):pass
        manager.lock=Barrier();manager.on_map(msg)
        self.assertIsNone(manager.grid);self.assertEqual(manager.revision,0)

    async def test_reset_success_clears_only_live_cache_and_filters_old_maps(self):
        manager=self.manager();manager.grid=('old-graph',0);manager.revision=900
        manager.last_sent=900;manager.last_preview=9
        manager.reset_client=Mock();manager.reset_client.service_is_ready.return_value=True
        manager.adapter.future=AsyncMock(return_value=NS(success=True,message=json.dumps({'generation':'new-engine'})))
        with patch.dict(sys.modules,{'std_srvs.srv':NS(Trigger=NS(Request=object))}):await manager.new_slam_session()
        self.assertIsNone(manager.grid);self.assertEqual(manager.revision,0);self.assertEqual(manager.last_sent,0)
        manager.on_map(self.grid(9));self.assertIsNone(manager.grid)

    async def test_failed_reset_cannot_silently_reuse_previous_map(self):
        manager=self.manager();manager.grid=('old-graph',0)
        manager.reset_client=Mock();manager.reset_client.service_is_ready.return_value=True
        manager.adapter.future=AsyncMock(return_value=NS(success=False,message='SLAM_RESET_FAILED'))
        with patch.dict(sys.modules,{'std_srvs.srv':NS(Trigger=NS(Request=object))}),self.assertRaises(StepError) as raised:
            await manager.new_slam_session()
        self.assertEqual(raised.exception.code,'SLAM_RESET_FAILED')

    def test_launch_keeps_vendor_hardware_and_managed_engine_separate(self):
        source=(ROOT/'pi/launch/mediassist_demo_slam.launch.py').read_text()
        self.assertIn('/slam/launch/include/robot.launch.py',source)
        self.assertIn('mediassist.slam_session',source);self.assertNotIn('/slam/launch/slam.launch.py',source)
        self.assertIn("'use_joy': 'false'",source)
        for topic in ('controller/cmd_vel','app/cmd_vel','odom_publisher:cmd_vel'):self.assertIn(topic,source)


if __name__=='__main__':unittest.main()
