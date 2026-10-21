#!/usr/bin/python3
"""Replay one prepared LIVO+BTC sequence at 1x over ROS transport."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client
import numpy as np


def run(prepared,output):
    output.mkdir(parents=True,exist_ok=False)
    with socket.socket() as s:
        s.bind(('127.0.0.1',0));port=s.getsockname()[1]
    uri='http://127.0.0.1:'+str(port);env=os.environ.copy();env['ROS_MASTER_URI']=uri
    command=['roslaunch','--port',str(port),'sa_livo','mapping_livo_loop.launch',
        'params_file:='+str(prepared/'parameters.yaml'),'camera_params_file:='+str(prepared/'camera.yaml'),
        'output_dir:='+str(output),'loop_enabled:=true','use_sim_time:=true']
    (output/'command.json').write_text(json.dumps(command,indent=2)+'\n')
    with open(output/'replay.log','w') as log:
        launch=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                if launch.poll() is not None: raise RuntimeError('Mapper exited before replay')
                try:
                    master=xmlrpc.client.ServerProxy(uri)
                    code,_,state=master.getSystemState('/livo_realtime_validator')
                    subscriptions=dict(state[1])
                    if code==1 and all('/laserMapping' in subscriptions.get(t,[]) for t in
                            ('/velodyne_points','/imu/data','/camera_1/image_rect')):break
                except (OSError,ConnectionError):pass
                time.sleep(.1)
            else:raise RuntimeError('Sensor subscriptions did not become ready')
            started=time.monotonic()
            play=subprocess.run(['rosbag','play','--clock','--rate','1.0',str(prepared/'normalized_livo.bag')],
                env=env,stdout=log,stderr=subprocess.STDOUT,timeout=360)
            elapsed=time.monotonic()-started
            if play.returncode:raise RuntimeError('rosbag play failed')
            if launch.poll() is not None:raise RuntimeError('Mapper exited during replay')
            time.sleep(2)
        finally:
            if launch.poll() is None:
                os.killpg(launch.pid,signal.SIGINT)
                try:launch.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid,signal.SIGTERM);launch.wait(timeout=10)
    text=(output/'replay.log').read_text(errors='replace')
    if 'SA-LIVO failed:' in text:raise RuntimeError('Realtime mapper failed')
    backend=json.loads((output/'backend_summary.json').read_text())
    q=np.atleast_1d(np.genfromtxt(output/'livo_quality.csv',delimiter=',',names=True))
    summary=dict(wall_replay_seconds=elapsed,backend=backend,frames=len(q),
        nonfinite_updates=int((q['finite']==0).sum()),
        visual_active_fraction=float(((q['q']>0)&(q['visual_valid']>0)).mean()))
    (output/'realtime_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();run(args.prepared.resolve(),args.output.resolve())
