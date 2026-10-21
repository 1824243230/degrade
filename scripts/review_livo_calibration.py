#!/usr/bin/python3
"""Create reproducible static projection overlays; these are not calibration optimization."""
import argparse
import importlib.util
import json
from pathlib import Path
import numpy as np
import yaml
import rosbag
import rospy
from sensor_msgs import point_cloud2


def review(directory):
    # The host also has a pip OpenCV without omnidir. Load the ROS/system ABI explicitly.
    spec = importlib.util.spec_from_file_location('cv2', '/usr/lib/python3/dist-packages/cv2.cpython-38-x86_64-linux-gnu.so')
    cv2 = importlib.util.module_from_spec(spec); spec.loader.exec_module(cv2)
    identity = json.loads((directory / 'input_identity.json').read_text())
    T = np.array(identity['T_CL'])
    camera = yaml.safe_load((directory / 'camera.yaml').read_text())['camera']
    out = directory / 'projection_review'; out.mkdir(exist_ok=True)
    records = []
    with rosbag.Bag(str(directory / 'normalized_livo.bag')) as bag:
        first, last = bag.get_start_time(), bag.get_end_time()
        for i, fraction in enumerate((.03, .25, .5, .75, .96)):
            t = first + fraction * (last-first)
            lidar, image = None, None
            for topic, message, _ in bag.read_messages(topics=['/velodyne_points', '/camera_1/image_rect'],
                    start_time=rospy.Time.from_sec(t), end_time=rospy.Time.from_sec(t+.5)):
                if topic == '/velodyne_points' and lidar is None: lidar = message
                if topic == '/camera_1/image_rect' and image is None: image = message
                if lidar is not None and image is not None: break
            if lidar is None or image is None: continue
            points = np.array(list(point_cloud2.read_points(lidar, field_names=('x','y','z'), skip_nans=True)))
            pc = points @ T[:3, :3].T + T[:3, 3]
            positive = pc[:, 2] > .5
            u = np.zeros(len(pc)); v = u.copy()
            u[positive] = camera['fx']*pc[positive, 0]/pc[positive, 2]+camera['cx']
            v[positive] = camera['fy']*pc[positive, 1]/pc[positive, 2]+camera['cy']
            valid = positive & (u>=0) & (u<image.width) & (v>=0) & (v<image.height)
            frame = np.ndarray((image.height, image.width, 3), np.uint8, buffer=image.data,
                               strides=(image.step, 3, 1)).copy()
            for x,y,z in zip(u[valid].astype(int),v[valid].astype(int),pc[valid,2]):
                value = min(255, int(z*12)); cv2.circle(frame,(x,y),1,(255-value,value,255),-1)
            cv2.imwrite(str(out / ('projection_%02d.png' % i)),frame)
            records.append(dict(lidar_time=lidar.header.stamp.to_sec(), image_time=image.header.stamp.to_sec(),
                delta_seconds=image.header.stamp.to_sec()-lidar.header.stamp.to_sec(),
                points=len(points), projected=int(valid.sum())))
    (out / 'review.json').write_text(json.dumps(dict(
        note='Static extrinsic projection without motion compensation; inspect straight/turn/static samples. Not an independent calibration accuracy measurement.',
        samples=records),indent=2)+'\n')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    review(parser.parse_args().directory.resolve())
