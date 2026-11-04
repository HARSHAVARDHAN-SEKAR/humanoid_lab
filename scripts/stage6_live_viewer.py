#!/usr/bin/env python3
"""LIVE 3D observer of MuJoCo qpos published from the ROS2 physics bridge.

Physics + ROS controller run in mujoco_bridge, NOT here. This process uses a
second MuJoCo model only to display the latest actual simulator configuration.
Simulation debug state is never used to control actuators.
"""
import math
import threading
import time
import numpy as np
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Bool


class LiveState(Node):
    def __init__(self):
        super().__init__('humanoid_live_visualizer')
        self.mutex = threading.Lock()
        self.qpos = None
        self.t = None
        self.wall_received = None
        self.mode = 'WAITING'
        self.safety = 'UNKNOWN'
        self.polygon = None
        self.last_status = 0.0
        self.create_subscription(Float64MultiArray,
            '/mujoco_bridge/debug_freebase_qpos', self._state, 100)
        self.create_subscription(String,
            '/com_balance_controller/mode', self._mode, 20)
        self.create_subscription(String,
            '/safety_supervisor/state', self._safety, 20)
        self.create_subscription(Bool,
            '/mujoco_bridge/support_polygon_valid', self._polygon, 20)

    def _state(self, msg):
        if len(msg.data) != 20:
            return
        vals = np.asarray(msg.data, dtype=float)
        if not np.isfinite(vals).all():
            return
        with self.mutex:
            self.t = float(vals[0])
            self.qpos = vals[1:].copy()
            self.wall_received = time.monotonic()

    def _mode(self, msg):
        self.mode = msg.data

    def _safety(self, msg):
        self.safety = msg.data

    def _polygon(self, msg):
        self.polygon = bool(msg.data)


def main():
    model = mujoco.MjModel.from_xml_path('/models/humanoid_biped.xml')
    if model.nq != 19:
        raise RuntimeError('Expected free-floating model nq=19, got '+str(model.nq))
    data = mujoco.MjData(model)
    rclpy.init()
    node = LiveState()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    print('Opening LIVE MuJoCo graphical window. Rotate with mouse; scroll to zoom.', flush=True)
    try:
        with mujoco.viewer.launch_passive(model, data,
                 show_left_ui=False, show_right_ui=False) as view:
            with view.lock():
                view.cam.azimuth = 140
                view.cam.elevation = -13
                view.cam.distance = 1.65
                view.cam.lookat[:] = (0.0, 0.0, 0.53)
            start = time.monotonic()
            last_frame_time = 0.0
            last_print = 0.0
            first_state_announced = False
            while view.is_running() and rclpy.ok():
                wall = time.monotonic()
                with node.mutex:
                    qp = None if node.qpos is None else node.qpos.copy()
                    sim_time = node.t
                    received = node.wall_received
                if qp is None:
                    if wall - start > 20:
                        raise RuntimeError('No debug qpos received within 20 s. Check ROS launch log.')
                    if wall - last_print > 2:
                        print('Waiting for /mujoco_bridge/debug_freebase_qpos ...', flush=True)
                        last_print = wall
                    time.sleep(.05)
                    continue
                if not first_state_announced:
                    first_state_announced = True
                    print('LIVE state received; robot is now visible in MuJoCo.',flush=True)
                    print('To apply a push, open a second Ubuntu terminal (instructions below).',flush=True)
                if wall - last_frame_time >= 1.0/30.0:
                    with view.lock():
                        data.qpos[:] = qp
                        mujoco.mj_forward(model, data)
                    view.sync()
                    last_frame_time = wall
                if wall - last_print > 2:
                    status = 'VALID' if node.polygon else ('INVALID' if node.polygon is False else 'UNKNOWN')
                    print('Sim t={:.2f}s  Controller={}  Safety={}  Polygon={}  Data age={:.2f}s'.format(
                        sim_time,node.mode,node.safety,status,wall-received),flush=True)
                    last_print = wall
                time.sleep(.005)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2)
        node.destroy_node()

if __name__ == '__main__':
    main()
