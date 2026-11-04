#!/usr/bin/env python3
"""View *actual* joint-state feedback from the Stage 4 supported ROS demo.

The physics and control loop run in the ROS MuJoCo bridge. This viewer only
reconstructs robot poses from /joint_states. The torso is FIXED by design;
this is supported stepping, NOT dynamically balanced or translating walking.
"""
import math
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String


class WalkObserver(Node):
    def __init__(self, model):
        super().__init__('supported_walk_visualizer')
        self.lock = threading.Lock()
        self.latest_qpos = None
        self.recv_time = None
        self.phase = 'WAITING'
        self.mode = 'WAITING'
        self.safety = 'UNKNOWN'
        self.steps_remaining = None
        self.joint_addresses = {}
        for jid in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if model.jnt_type[jid] == mujoco.mjtJoint.mjJNT_HINGE:
                self.joint_addresses[name] = int(model.jnt_qposadr[jid])
        if len(self.joint_addresses) != 12:
            raise RuntimeError('Expected 12 controllable hinge joints')
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 50)
        self.create_subscription(String, '/motion_controller/mode', self.on_mode, 20)
        self.create_subscription(String, '/mission_manager/phase', self.on_phase, 20)
        self.create_subscription(String, '/safety_supervisor/state', self.on_safety, 20)
        self.create_subscription(Int32, '/gait_planner/steps_remaining', self.on_steps, 20)

    def on_joint_states(self, msg):
        if len(msg.name) != len(msg.position):
            return
        positions = dict(zip(msg.name, msg.position))
        if any(n not in positions or not math.isfinite(positions[n])
               for n in self.joint_addresses):
            return
        q = np.zeros(len(self.joint_addresses), dtype=float)
        for name, adr in self.joint_addresses.items():
            q[adr] = float(positions[name])
        with self.lock:
            self.latest_qpos = q
            self.recv_time = time.monotonic()

    def on_mode(self, msg):
        self.mode = msg.data

    def on_phase(self, msg):
        self.phase = msg.data

    def on_safety(self, msg):
        self.safety = msg.data

    def on_steps(self, msg):
        self.steps_remaining = msg.data


def main():
    model = mujoco.MjModel.from_xml_path('/models/humanoid_biped_supported.xml')
    if model.nq != 12:
        raise RuntimeError(f'Expected supported model nq=12, got {model.nq}')
    data = mujoco.MjData(model)
    rclpy.init()
    node = WalkObserver(model)
    worker = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    worker.start()
    print('Stage 4 SUPPORTED stepping: torso is fixed to the world.', flush=True)
    print('Opening 3D MuJoCo window. Start your desktop screen recording.', flush=True)
    print('Then run the STEP TRIGGER from a second Ubuntu terminal.', flush=True)
    started = time.monotonic()
    last_log = 0.0
    got_feedback = False
    try:
        with mujoco.viewer.launch_passive(model, data,
              show_left_ui=False, show_right_ui=False) as viewer:
            with viewer.lock():
                viewer.cam.azimuth = 130
                viewer.cam.elevation = -14
                viewer.cam.distance = 1.70
                viewer.cam.lookat[:] = (0.0, 0.0, 0.42)
            while viewer.is_running() and rclpy.ok():
                now = time.monotonic()
                with node.lock:
                    q = None if node.latest_qpos is None else node.latest_qpos.copy()
                    recv = node.recv_time
                if q is not None:
                    if not got_feedback:
                        print('ROS /joint_states received; viewer is live.', flush=True)
                        got_feedback = True
                    with viewer.lock():
                        data.qpos[:] = q
                        mujoco.mj_forward(model, data)
                    viewer.sync()
                elif now-started > 20:
                    raise RuntimeError('No /joint_states after 20 seconds: inspect /tmp/stage4_walk_launch.log')
                if now-last_log > 2:
                    age = None if recv is None else round(now-recv, 3)
                    print(f'Phase={node.phase} | mode={node.mode} | '
                          f'safety={node.safety} | steps_remaining={node.steps_remaining} | '
                          f'feedback_age_s={age}', flush=True)
                    last_log = now
                time.sleep(1/30)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        worker.join(timeout=2)
        node.destroy_node()


if __name__ == '__main__':
    main()
