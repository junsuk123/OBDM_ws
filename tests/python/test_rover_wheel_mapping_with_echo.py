#!/usr/bin/env python3
"""Per-wheel force test that reads Gazebo joint_state immediately after each publish.

This script publishes a force to one wheel, then captures one joint_state
message from Gazebo and prints the `axis1.velocity` for that wheel, so you
can determine whether the applied force produced positive or negative rotation.
"""
import os
import subprocess
import time
import re

WHEELS = [
    ('motor_0', '/model/r1_rover/joint/motor_0/cmd_force'),
    ('motor_1', '/model/r1_rover/joint/motor_1/cmd_force'),
    ('motor_2', '/model/r1_rover/joint/motor_2/cmd_force'),
    ('motor_3', '/model/r1_rover/joint/motor_3/cmd_force'),
]
FORCE = float(os.getenv('OBDM_ROVER_FORCE_GAIN', '2.0'))


def publish_force(topic, value):
    cmd = ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double', '-p', f'data: {float(value)}']
    subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def read_joint_state():
    cmd = ['gz', 'topic', '-e', '-t', '/world/runway/model/r1_rover/joint_state', '-n', '1']
    p = subprocess.run(cmd, check=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return p.stdout


def extract_velocity_for_joint(joint_name, joint_state_text):
    # crude search for the joint block and velocity line
    # locate 'joint {' blocks and parse name and axis1 velocity
    blocks = re.split(r'\njoint \{', '\n' + joint_state_text)
    for b in blocks:
        if f'name: "{joint_name}"' in b:
            # find velocity: line
            m = re.search(r'velocity:\s*([-+e0-9.\.]+)', b)
            if m:
                return float(m.group(1))
    return None


def zero_all():
    for _, t in WHEELS:
        publish_force(t, 0.0)


def run_test():
    print('Per-wheel mapping test with immediate joint_state read')
    for name, topic in WHEELS:
        print(f'-- Testing {name} ({topic}) : +{FORCE}')
        publish_force(topic, +FORCE)
        time.sleep(0.2)
        js = read_joint_state()
        vel = extract_velocity_for_joint(name, js)
        print(f'   observed velocity for {name}: {vel}')
        publish_force(topic, 0.0)
        time.sleep(0.2)
        print(f'-- Testing {name} ({topic}) : -{FORCE}')
        publish_force(topic, -FORCE)
        time.sleep(0.2)
        js = read_joint_state()
        vel = extract_velocity_for_joint(name, js)
        print(f'   observed velocity for {name}: {vel}')
        publish_force(topic, 0.0)
        time.sleep(0.2)
    print('Done; zeroed forces')
    zero_all()


if __name__ == '__main__':
    run_test()
