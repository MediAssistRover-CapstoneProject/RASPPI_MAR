"""Own only the SLAM engine; reset its RAM graph without restarting robot drivers."""
import argparse
import copy
import json
from pathlib import Path
import signal
import subprocess
import time
import uuid
import sys
import yaml


def process_tools():
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools'))
    from robot_stack import proc,terminate_tree
    return proc,terminate_tree


def engine_parameters(source,mapping,ros):
    from .scan_normalizer import configuration, TOPIC
    scan_topic = TOPIC if configuration(mapping)['enabled'] else ros['scan']
    params=copy.deepcopy(source.get('/**',source.get('slam_toolbox',{})).get('ros__parameters',{}))
    if not params:raise ValueError('VENDOR_SLAM_PARAMETERS_MISSING')
    for key in ('map_file_name','map_start_pose','map_start_at_dock'):params.pop(key,None)
    params.update(use_sim_time=False,mode='mapping',map_frame=ros['map_frame'],odom_frame=ros.get('odom_frame','odom'),
        base_frame=ros['base_frame'],scan_topic=scan_topic,resolution=mapping['slam_resolution_m'],
        minimum_travel_distance=mapping['slam_min_travel_m'],minimum_travel_heading=mapping['slam_min_heading_rad'],
        minimum_time_interval=mapping['slam_min_interval_sec'],map_update_interval=mapping['slam_map_interval_sec'],
        transform_timeout=.5,tf_buffer_duration=30.,scan_queue_size=10)
    return {'slam_toolbox':{'ros__parameters':params}}


class Engine:
    def __init__(self,params_path,log_path):
        self.params_path,self.log_path=Path(params_path),Path(log_path)
        self.process=None
        self.identity=None
        self.generation=None

    def start(self):
        if self.process and self.process.poll() is None:raise RuntimeError('SLAM_ENGINE_ALREADY_RUNNING')
        self.generation=str(uuid.uuid4())
        self.log_path.parent.mkdir(parents=True,exist_ok=True)
        with self.log_path.open('ab') as log:
            log.write(('\nSLAM_GENERATION '+self.generation+'\n').encode());log.flush()
            self.process=subprocess.Popen(['ros2','run','slam_toolbox','sync_slam_toolbox_node','--ros-args',
                '-r','__node:=slam_toolbox','--params-file',str(self.params_path)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        self.identity=process_tools()[0](self.process.pid)
        if self.identity is None:raise RuntimeError('SLAM_ENGINE_START_FAILED')
        return self.generation

    def stop(self):
        child=self.process
        if child is None:return
        if self.identity is not None:
            process_tools()[1](self.identity,grace=4.)
        child.wait(timeout=2.)
        self.process=None
        self.identity=None

    def restart(self):
        self.stop()
        return self.start()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True)
    args=parser.parse_args()
    from .worker_config import load
    cfg,hardware=load(args.config)
    mapping=cfg['mapping']
    source=Path('/home/ubuntu/ros2_ws/src/slam/config/slam.yaml')
    if not source.is_file():
        from ament_index_python.packages import get_package_share_directory
        source=Path(get_package_share_directory('slam'))/'config/slam.yaml'
    params=engine_parameters(yaml.safe_load(source.read_text()),mapping,hardware['ros'])
    folder=Path(args.config).resolve().parent.parent/'state/stack'
    folder.mkdir(parents=True,exist_ok=True)
    params_path=folder/'slam-session.yaml'
    params_path.write_text(yaml.safe_dump(params),encoding='utf-8')
    engine=Engine(params_path,folder/'slam-engine.log')
    from .scan_normalizer import ManagedNormalizer, configuration
    normalizer = ManagedNormalizer(args.config,folder/'scan-normalizer.log',process_tools()) if configuration(mapping)['enabled'] else None
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_srvs.srv import Trigger
    from std_msgs.msg import String
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('mediassist_slam_session')
    gate=[None,0.]
    stopping=[False]
    def stop(*_):stopping[0]=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,stop)
    def seen_gate(msg):
        try:gate[:]=[json.loads(msg.data),time.monotonic()]
        except ValueError:gate[:]=[None,0.]
    node.create_subscription(String,'/mediassist/safety_state',seen_gate,10)
    def reset(request,response):
        if gate[0] is None or time.monotonic()-gate[1]>.6 or gate[0].get('moving_command') is not False:
            response.success=False;response.message='SLAM_RESET_REQUIRES_STOPPED_GATE';return response
        try:
            generation=engine.restart()
            response.success=True;response.message=json.dumps({'generation':generation})
            node.get_logger().info('NEW_SLAM_SESSION '+generation+' resolution='+str(mapping['slam_resolution_m']))
        except Exception as exc:
            response.success=False;response.message='SLAM_RESET_FAILED: '+type(exc).__name__
        return response
    node.create_service(Trigger,mapping['reset_service'],reset)
    try:
        if normalizer: normalizer.start()
        engine.start()
        node.get_logger().info('SLAM_SESSION_READY reset_service='+mapping['reset_service'])
        while rclpy.ok() and not stopping[0]:
            rclpy.spin_once(node,timeout_sec=.1)
            if normalizer: normalizer.check()
            if engine.process is None or engine.process.poll() is not None:raise RuntimeError('SLAM_ENGINE_EXITED')
    finally:
        try:
            engine.stop()
        finally:
            if normalizer: normalizer.stop()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
