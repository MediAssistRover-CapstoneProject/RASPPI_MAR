"""Local keyboard only; worker owns the motion lease and safety checks."""
import argparse
import json
import select
import sys
import termios
import time
import tty
from uuid import UUID


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session-id',required=True)
    args=p.parse_args()
    UUID(args.session_id)
    if not sys.stdin.isatty(): raise SystemExit('Use docker exec -it and a terminal')
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    node=rclpy.create_node('mediassist_mapping_keyboard')
    pub=node.create_publisher(String,'/mediassist/mapping/manual',1)
    saved=termios.tcgetattr(sys.stdin)
    def send(x=0.,z=0.):
        pub.publish(String(data=json.dumps({'mapping_session_id':args.session_id,'linear_x':x,'angular_z':z})))
    try:
        tty.setcbreak(sys.stdin.fileno())
        print('W: forward 0.05 m/s; A/D: turn; SPACE: stop; Q: exit. Hold key to move.')
        while rclpy.ok():
            if select.select([sys.stdin],[],[],.05)[0]:
                key=sys.stdin.read(1).lower()
                if key in ('q','\x03'): break
                x,z={'w':(.05,0.),'a':(0.,.2),'d':(0.,-.2)}.get(key,(0.,0.))
                send(x,z)
            rclpy.spin_once(node,timeout_sec=0)
    finally:
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,saved)
        for _ in range(3): send(); time.sleep(.05)
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
