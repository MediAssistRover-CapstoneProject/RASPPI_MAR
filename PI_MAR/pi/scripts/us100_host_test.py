#!/usr/bin/env python3
"""Read one US100 using lgpio only. No ROS node or topics are created.

May also run inside MentorPi with .venv/bin/python, without ROS imports.
Stop us100_dual_ros2.py before running this diagnostic on the same GPIO.
"""
import argparse
import time
from us100_dual_ros2 import US100Channel, choose_chip, pins_for


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=['left', 'right'], required=True)
    parser.add_argument('--gpiochip', type=int, default=None)
    parser.add_argument('--samples', type=int, default=0, help='0: keep running until Ctrl+C')
    args = parser.parse_args()
    if args.samples < 0:
        parser.error('--samples must be >=0')
    import lgpio
    chip = choose_chip(lgpio, args.gpiochip)
    handle, sensor = lgpio.gpiochip_open(chip), None
    trig, echo = pins_for(args.side)
    try:
        sensor = US100Channel(lgpio, handle, args.side, trig, echo)
        print(f'US100 {args.side}: TRIG BCM{trig}, ECHO BCM{echo}, gpiochip{chip}', flush=True)
        print('Direct GPIO only; no ROS topic. NO_ECHO does not mean clear space.', flush=True)
        time.sleep(0.1)
        count = 0
        while args.samples == 0 or count < args.samples:
            result = sensor.measure()
            detail = f'{result.distance_m * 100:.1f} cm' if result.distance_m is not None else '-'
            print(f'{args.side}: {result.status}; distance={detail}', flush=True)
            count += 1
            time.sleep(0.16)
    except KeyboardInterrupt:
        pass
    finally:
        if sensor is not None:
            sensor.close()
        lgpio.gpiochip_close(handle)


if __name__ == '__main__':
    main()
