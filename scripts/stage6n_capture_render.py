#!/usr/bin/env python3
"""Capture actual free-base MuJoCo qpos over ROS2 and replay into MP4.

Recording reads /mujoco_bridge/debug_freebase_qpos only: a SIMULATION
GROUND-TRUTH visualization topic, not a real sensor. The already validated
CoM controller continues to use its existing estimator topics.

Two subcommands are available: capture and render. Capture only stores
states and control-state labels; render replays poses without rerunning the
physics or controller. Replaying qpos reproduces the visible poses but not
contact forces or dynamically integrated forces inside the renderer.
"""
import argparse
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np


def capture(args):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, Float64, String, Bool

    class Recorder(Node):
        def __init__(self):
            super().__init__('stage6n_movie_recorder')
            self.states = []
            self.times = []
            self.modes = []
            self.safeties = []
            self.polygons = []
            self.mode = None
            self.safety = None
            self.polygon = None
            self.base_time = None
            self.push_time = None
            self.last_sim_time = None
            self.error = None
            self.finished = False
            self.created = time.monotonic()
            self.last_state_wall = None
            self.create_subscription(Float64MultiArray,
                '/mujoco_bridge/debug_freebase_qpos', self.on_state, 100)
            self.create_subscription(String,
                '/com_balance_controller/mode', self.on_mode, 30)
            self.create_subscription(String,
                '/safety_supervisor/state', self.on_safety, 30)
            self.create_subscription(Bool,
                '/mujoco_bridge/support_polygon_valid', self.on_polygon, 30)
            self.push_pub = self.create_publisher(
                Float64, '/mujoco_bridge/inject_torso_push_x', 10)

        def on_mode(self, msg):
            self.mode = msg.data

        def on_safety(self, msg):
            self.safety = msg.data

        def on_polygon(self, msg):
            self.polygon = bool(msg.data)

        def on_state(self, msg):
            values = list(msg.data)
            if len(values) != 20 or not all(math.isfinite(v) for v in values):
                self.error = 'Invalid 20-float MuJoCo state packet'
                self.finished = True
                return
            sim_t = values[0]
            self.last_state_wall = time.monotonic()
            if self.last_sim_time is not None and sim_t < self.last_sim_time:
                self.error = 'MuJoCo time moved backward'
                self.finished = True
                return
            self.last_sim_time = sim_t
            if self.base_time is None:
                if (self.mode == 'ACTIVE_COM_PD' and self.safety == 'STAND'
                        and self.push_pub.get_subscription_count() == 1):
                    self.base_time = sim_t
                    print('ROS loop ACTIVE_COM_PD / STAND; capturing baseline...',flush=True)
                return
            t = sim_t - self.base_time
            self.times.append(t)
            self.states.append(values[1:])
            self.modes.append(self.mode or 'UNKNOWN')
            self.safeties.append(self.safety or 'UNKNOWN')
            self.polygons.append(-1 if self.polygon is None else int(self.polygon))
            if self.push_time is None and t >= 2.0:
                if self.mode != 'ACTIVE_COM_PD' or self.safety != 'STAND':
                    self.error='Controller not ACTIVE_COM_PD/STAND at push time'
                    self.finished=True
                    return
                self.push_time = t
                self.push_pub.publish(Float64(data=float(args.force)))
                print(f'Published {args.force:+g} N push at t={t:.3f} sim s', flush=True)
            if self.push_time is not None and t >= self.push_time + 6.5:
                self.finished = True

    rclpy.init()
    n=Recorder()
    try:
        while rclpy.ok() and not n.finished:
            rclpy.spin_once(n,timeout_sec=0.05)
            wall_elapsed = time.monotonic()-n.created
            if wall_elapsed > 45.0:
                n.error = 'Recording timed out waiting for running simulation'
                break
            if n.base_time is not None and n.last_state_wall is not None:
                if time.monotonic()-n.last_state_wall > 1.0:
                    n.error = 'Simulator state stream stopped'; break
        if n.error:
            raise RuntimeError(n.error)
        if n.push_time is None or len(n.times) < 250:
            raise RuntimeError('Incomplete recording or missing push')
        out=Path(args.output)
        out.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(out,
            times=np.asarray(n.times,dtype=np.float64),
            qpos=np.asarray(n.states,dtype=np.float64),
            modes=np.asarray(n.modes,dtype='U32'),
            safeties=np.asarray(n.safeties,dtype='U32'),
            polygons=np.asarray(n.polygons,dtype=np.int8),
            push_time=np.float64(n.push_time),
            force=np.float64(args.force))
        npts = len(n.times)
        print(f'Saved {npts} full-body simulation poses to {out}',flush=True)
        print('Safety states:', sorted(set(n.safeties)),flush=True)
        print('Controller modes:', sorted(set(n.modes)),flush=True)
        print('Sampled invalid polygons:', sum(p==0 for p in n.polygons),flush=True)
        print('NOTE: this is a ROS debug-state replay, NOT an independent physics run.',flush=True)
        print('STAGE 6N ROS RECORDING COMPLETE',flush=True)
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


def render(args):
    # The env must be set *before* importing mujoco (handled by shell).
    import mujoco
    from PIL import Image, ImageDraw, ImageFont
    with np.load(args.input,allow_pickle=False) as z:
        times=z['times']; qpos=z['qpos']; modes=z['modes']
        safeties=z['safeties']; polygons=z['polygons']
        push_time=float(z['push_time']); force=float(z['force'])
    if qpos.ndim != 2 or qpos.shape[1] != 19:
        raise ValueError('Expected free-base qpos shape (N,19)')
    root=Path(__file__).resolve().parents[1]
    model_path=root/'src/humanoid_sim/mujoco_models/humanoid_biped.xml'
    if not model_path.is_file():
        # Docker mounts model assets as a separate read-only directory.
        model_path=Path('/models/humanoid_biped.xml')
    model=mujoco.MjModel.from_xml_path(str(model_path))
    data=mujoco.MjData(model)
    model.vis.global_.offwidth=1024
    model.vis.global_.offheight=768
    torso=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'torso')
    mass=float(model.body_mass.sum())
    w,h,strip=960,540,120
    camera=mujoco.MjvCamera()
    camera.type=mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:]=[0.0,0.0,0.53]
    camera.distance=1.9
    camera.azimuth=135
    camera.elevation=-12
    fontfile='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    try:
        font=ImageFont.truetype(fontfile,19)
        small=ImageFont.truetype(fontfile,15)
    except OSError:
        font=ImageFont.load_default(); small=font

    # Compute full trace CoM and torso tilt from captured qpos; these are
    # replayed simulator poses, not a second physics rollout.
    com_x=np.empty(len(times))
    tilt=np.empty(len(times))
    for i,qp in enumerate(qpos):
        data.qpos[:]=qp
        mujoco.mj_forward(model,data)
        com_x[i]=float((model.body_mass[:,None]*data.xipos).sum(axis=0)[0]/mass)
        R=data.xmat[torso].reshape(3,3)
        tilt[i]=math.degrees(math.acos(float(np.clip(R[2,2],-1.0,1.0))))
    pre=(times>0.5)&(times<push_time-0.1)
    reference=float(np.mean(com_x[pre])) if pre.any() else float(com_x[0])
    duration=float(times[-1])
    fps=30
    count=int(math.floor(duration*fps))
    output=Path(args.output); output.parent.mkdir(parents=True, exist_ok=True)
    cmd=['ffmpeg','-hide_banner','-loglevel','error','-y',
         '-f','rawvideo','-pixel_format','rgb24',
         '-video_size',f'{w}x{h+strip}','-framerate',str(fps),
         '-i','pipe:0','-c:v','libx264','-preset','veryfast',
         '-crf','24','-pix_fmt','yuv420p','-movflags','+faststart',str(output)]
    proc=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=subprocess.PIPE)
    try:
        with mujoco.Renderer(model,width=w,height=h) as renderer:
            for j in range(count):
                t=j/fps
                idx=int(np.searchsorted(times,t,side='left'))
                idx=min(max(idx,0),len(times)-1)
                data.qpos[:]=qpos[idx]
                mujoco.mj_forward(model,data)
                renderer.update_scene(data,camera=camera)
                rgb=renderer.render()
                frame=Image.new('RGB',(w,h+strip),(15,20,28))
                frame.paste(Image.fromarray(rgb),(0,0))
                draw=ImageDraw.Draw(frame)
                draw.rectangle((0,0,w,42),fill=(15,20,28))
                direction='FORWARD' if force>0 else 'BACKWARD'
                draw.text((15,9),f'HUMANOID-LAB  |  ROS 2 + MuJoCo  |  {abs(force):g} N {direction}',font=font,fill=(240,244,247))
                if push_time <= t < push_time+0.1:
                    draw.rectangle((w-250,50,w-14,88),fill=(180,60,45))
                    draw.text((w-237,59),'PUSH APPLIED',font=font,fill=(255,255,255))
                dx=(com_x[idx]-reference)*1000.0
                pol='VALID' if polygons[idx]==1 else ('DEGENERATE' if polygons[idx]==0 else 'UNKNOWN')
                info=f't={t:5.2f}s | CoM X shift={dx:+6.1f} mm | tilt={tilt[idx]:.2f} deg'
                draw.text((17,h+8),info,font=font,fill=(238,241,249))
                states=f'Controller: {modes[idx]} | Safety: {safeties[idx]} | Contact polygon: {pol}'
                draw.text((17,h+39),states,font=small,fill=(180,220,235))
                x0,x1=20,w-20; y=h+84
                draw.line([(x0,y),(x1,y)],fill=(80,93,111),width=5)
                xp=int(x0+(x1-x0)*min(1.0,t/duration))
                xpush=int(x0+(x1-x0)*push_time/duration)
                draw.ellipse((xp-7,y-7,xp+7,y+7),fill=(255,255,255))
                draw.line([(xpush,y-11),(xpush,y+11)],fill=(255,128,83),width=4)
                draw.text((x0,y+13),'0 s',font=small,fill=(180,180,180))
                draw.text((x1-44,y+13),f'{duration:.1f} s',font=small,fill=(180,180,180))
                if j == max(0, int((push_time+0.2)*fps)):
                    frame.save(output.with_suffix('.png'))
                proc.stdin.write(frame.tobytes())
                if j%90==0:
                    print(f'Rendered {j}/{count} video frames',flush=True)
        proc.stdin.close()
        err=proc.stderr.read().decode('utf-8','replace')
        code=proc.wait()
        if code!=0:
            raise RuntimeError('ffmpeg failed: '+err[-1500:])
    except Exception:
        if proc.poll() is None:
            proc.kill(); proc.wait()
        raise
    print(f'VIDEO CREATED: {output} ({output.stat().st_size/1024**2:.2f} MB)',flush=True)
    print('Peak |CoM X shift| replayed:',f'{max(abs(com_x-reference))*1000:.2f} mm',flush=True)
    print('Peak torso tilt replayed:',f'{max(tilt):.3f} deg',flush=True)
    print('STAGE 6N VIDEO COMPLETE',flush=True)


def main():
    p=argparse.ArgumentParser()
    sub=p.add_subparsers(dest='action',required=True)
    c=sub.add_parser('capture')
    c.add_argument('--force',type=float,required=True)
    c.add_argument('--output',required=True)
    r=sub.add_parser('render')
    r.add_argument('--input',required=True)
    r.add_argument('--output',required=True)
    a=p.parse_args()
    (capture if a.action=='capture' else render)(a)

if __name__=='__main__':main()
