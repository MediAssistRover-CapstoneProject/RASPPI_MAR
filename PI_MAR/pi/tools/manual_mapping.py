"""Slow keyboard commands through the same gate, after physical bench checks."""
import argparse
import asyncio
from pathlib import Path
import select
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mediassist.configuration import load_config
from mediassist.cli import instance_lock
from mediassist.store import Store
from mediassist.adapters import RemoteServices


async def run(args, cfg):
    import termios
    import tty
    from geometry_msgs.msg import Twist
    from mediassist.ros_adapter import RosAdapter
    if not sys.stdin.isatty(): raise ValueError('Run in an interactive Docker terminal with -it')
    if (cfg['allowed_workflows'] or cfg['max_speed_mps'] > .05
            or not cfg['navigation'].get('operator_test_notes','').startswith('MANUAL_MAPPING:')):
        raise ValueError('Use the explicit manual-profile after bench tests')
    adapter = RosAdapter(cfg, RemoteServices())
    pub = adapter.node.create_publisher(Twist, cfg['ros']['raw_cmd_vel'], 10)
    settings = termios.tcgetattr(sys.stdin)
    command, last_key, checked_at = (0., 0.), 0., 0.
    try:
        await asyncio.sleep(2)
        adapter._check_graph()
        if not adapter.health()['sensors_ready']: raise ValueError('US100 and LiDAR must be fresh before manual mapping')
        print('W forward 0.05 m/s; A/D turn 0.2 rad/s; SPACE stop; Q/Ctrl+C quit. No key repeats for 0.25s => zero.')
        tty.setcbreak(sys.stdin.fileno())
        while True:
            now = time.monotonic()
            if select.select([sys.stdin], [], [], 0)[0]:
                key=sys.stdin.read(1).lower()
                if key in ('q','\x03'): break
                command = {'w': (min(.05,cfg['max_speed_mps']),0.),'a':(0.,.2),'d':(0.,-.2)}.get(key,(0.,0.))
                last_key=now
            if now-checked_at > .2:
                adapter._check_graph()
                checked_at=now
            if now-last_key > .25 or not adapter.health()['sensors_ready']: command=(0.,0.)
            msg=Twist(); msg.linear.x,msg.angular.z=command
            pub.publish(msg)
            adapter.lease_message(any(command), cfg['max_speed_mps'] if any(command) else 0.)
            await asyncio.sleep(.05)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        for _ in range(5):
            pub.publish(Twist()); adapter.lease_message(False); await asyncio.sleep(.05)
        await adapter.stop(); adapter.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--operator-present', action='store_true', required=True)
    a=p.parse_args(); cfg=load_config(a.config)
    state=Path(cfg['state_path']); state.parent.mkdir(parents=True,exist_ok=True)
    with instance_lock(state):
        store=Store(state,recover=True)
        try:
            if any(s=='RECOVERY_REQUIRED' for s in store.states().values()): raise ValueError('Reconcile journal first')
            asyncio.run(run(a,cfg))
        finally: store.close()


if __name__=='__main__': main()
