#!/usr/bin/env python3
"""Sequential per-wheel force test for r1_rover.

This script publishes gz.msgs.Double to each wheel command topic using
`gz topic -p`, mirroring `rover_cmd_vel_to_wheels.py`'s mechanism.

Usage: run while simulation is running. Watch Gazebo or `/joint_states`
for wheel rotation direction. Adjust `FORCE` via env `OBDM_ROVER_FORCE_GAIN`.
"""
import os
import subprocess
import time

WHEELS = [
    '/model/r1_rover/joint/motor_0/cmd_force',
    '/model/r1_rover/joint/motor_1/cmd_force',
    '/model/r1_rover/joint/motor_2/cmd_force',
    '/model/r1_rover/joint/motor_3/cmd_force',
]
FORCE = float(os.getenv('OBDM_ROVER_FORCE_GAIN', '2.0'))
DURATION = 1.0


def publish_force(topic, value):
    cmd = ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double', '-p', f'data: {float(value)}']
    try:
        subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        print(f'Publish error: {e}')


def zero_all():
    for t in WHEELS:
        publish_force(t, 0.0)


def run_test():
    print('Starting per-wheel mapping test')
    print(f'Using FORCE={FORCE}')
    time.sleep(0.5)
    for idx, topic in enumerate(WHEELS):
        print(f'== Wheel {idx} => {topic} : +{FORCE} for {DURATION}s')
        publish_force(topic, +FORCE)
        time.sleep(DURATION)
        publish_force(topic, 0.0)
        time.sleep(0.2)
        print(f'== Wheel {idx} => {topic} : -{FORCE} for {DURATION}s')
        publish_force(topic, -FORCE)
        time.sleep(DURATION)
        publish_force(topic, 0.0)
        time.sleep(0.5)
    print('Test finished — zeroing all forces')
    zero_all()


if __name__ == '__main__':
    run_test()
