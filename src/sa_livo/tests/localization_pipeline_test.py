#!/usr/bin/env python3
"""Synthetic ROS1 replay through the actual mapper, including tracking loss.

No external recordings or sensor hardware are needed. This is an integration
regression test, not a real-scene accuracy measurement.
"""
import copy
import os
from pathlib import Path
import subprocess
import signal
import tempfile
import time
import unittest

import numpy as np
import rosbag
import roslib.packages
import rospkg
import rospy
import rostest
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, PointField, Image
from std_msgs.msg import Header
from sa_livo.msg import LocalizationStatus
import yaml


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*cross + (1-np.cos(angle))*cross.dot(cross)


def scene():
    points = []
    for i in range(64):
        for j in range(42):
            x, y = (i-32)*.15, (j-21)*.15
            points.append((x, y, -1.7+.04*np.sin(x*1.7)))
            points.append((5.2+.03*np.sin(y*2), x*.7, (j-21)*.1))
            if i < 45:
                points.append((x, -4.3, (j-21)*.11+.02*np.sin(x)))
    for i in range(24):
        for j in range(24):
            a, b = i*.11, j*.13
            points.append((-1.3+np.cos(a)*np.sin(b), 1.7+np.sin(a)*np.sin(b), .4+np.cos(b)))
    return np.asarray(points, dtype=np.float32)


def merge(target, extra):
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            merge(target[key], value)
        else:
            target[key] = value


class PipelineTest(unittest.TestCase):
    def test_direct_bag_exits_without_tail_imu_coverage(self):
        package=Path(rospkg.RosPack().get_path('sa_livo'))
        with tempfile.TemporaryDirectory(prefix='sa_livo_tail_') as directory:
            root=Path(directory); points=scene(); map_path=root/'map.pcd'
            with map_path.open('wb') as stream:
                stream.write(('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                    'WIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA binary\n'%(len(points),len(points))).encode())
                stream.write(points.astype('<f4').tobytes())
            (root/'map.pcd.yaml').write_text(yaml.safe_dump(dict(units='m',frame_id='map',
                version='tail-test',calibration_id='tail-test',gravity=[0.,0.,-9.81])))
            bag_path=root/'tail.bag'
            imu=Imu();imu.header.stamp=rospy.Time.from_sec(1000.);imu.linear_acceleration.z=9.81
            scan=pc2.create_cloud_xyz32(Header(stamp=rospy.Time.from_sec(1000.1),frame_id='lidar'),points)
            with rosbag.Bag(str(bag_path),'w') as bag:
                bag.write('/test/imu',imu,imu.header.stamp)
                bag.write('/test/points',scan,scan.header.stamp)
            params=yaml.safe_load((package/'config/avia.yaml').read_text())
            merge(params,yaml.safe_load((package/'config/localization.yaml').read_text()))
            params['common'].update(img_en=0,lid_topic='/test/points',imu_topic='/test/imu',direct_bag_path=str(bag_path))
            params['preprocess'].update(lidar_type=4,point_filter_num=1,blind=.1)
            params['localization'].update(map_path=str(map_path),metadata_path=str(map_path)+'.yaml',calibration_id='tail-test')
            params['evo']['pose_output_en']=False
            rospy.set_param('/laserMapping',params);rospy.set_param('/use_sim_time',False)
            executable=roslib.packages.find_node('sa_livo','fastlivo_mapping')[0]
            with (root/'mapper.log').open('w') as log:
                process=subprocess.Popen([executable,'__name:=laserMapping'],stdout=log,stderr=subprocess.STDOUT)
                try:
                    self.assertEqual(process.wait(timeout=10),0)
                finally:
                    if process.poll() is None:
                        process.kill();process.wait()
            self.assertIn('trailing LiDAR scan has no IMU coverage',(root/'mapper.log').read_text())

    def test_startup_and_recovery_without_camera(self):
        self.run_pipeline(with_camera=False, live=False)

    def test_joint_pipeline_through_rosbag_play(self):
        self.run_pipeline(with_camera=True, live=True)

    def run_pipeline(self, with_camera, live):
        package = Path(rospkg.RosPack().get_path('sa_livo'))
        statuses, poses = [], []
        subscriptions = [
            rospy.Subscriber('/localization/status', LocalizationStatus, statuses.append, queue_size=1000),
            rospy.Subscriber('/localization/pose', PoseWithCovarianceStamped, poses.append, queue_size=1000),
        ]
        with tempfile.TemporaryDirectory(prefix='sa_livo_pipeline_') as directory:
            root = Path(directory)
            points = scene()
            map_path = root/'map.pcd'
            with map_path.open('wb') as stream:
                stream.write(('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                              'WIDTH %d\nHEIGHT 1\nPOINTS %d\nDATA binary\n' % (len(points), len(points))).encode())
                stream.write(points.astype('<f4').tobytes())
            gravity = np.array([1., 2., -9.])
            gravity *= 9.81/np.linalg.norm(gravity)
            with (root/'map.pcd.yaml').open('w') as stream:
                yaml.safe_dump(dict(units='m', frame_id='map', version='synthetic-test',
                                    calibration_id='synthetic-calibration', gravity=gravity.tolist()), stream)
            R_map_lidar = rotation([0, 0, 1], .35).dot(rotation([1, 0, 0], .1))
            T_map_lidar = np.array([2., -1., .5])
            R_imu_lidar = rotation([0, 1, 0], .07)
            T_imu_lidar = np.array([.12, .03, -.04])
            R_map_imu = R_map_lidar.dot(R_imu_lidar.T)
            expected = T_map_lidar-R_map_imu.dot(T_imu_lidar)
            source = (points-T_map_lidar).dot(R_map_lidar)
            acceleration = R_map_imu.T.dot(-gravity)
            fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1), PointField('rgb', 12, PointField.UINT32, 1)]
            scan = pc2.create_cloud(Header(frame_id='lidar'), fields,
                                    [(float(x), float(y), float(z), 0x8090a0) for x, y, z in source]+
                                    [(float('nan'),float('nan'),float('nan'),0)])
            corrupt = copy.deepcopy(scan)
            raw = np.frombuffer(corrupt.data, dtype=np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])).copy()
            raw['z'] += 20
            corrupt.data = raw.tobytes()
            imu = Imu()
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = acceleration
            bag_path = root/'test.bag'
            image=Image(height=1024,width=1280,encoding='mono8',step=1280,data=bytes([128])*(1024*1280))
            with rosbag.Bag(str(bag_path), 'w') as bag:
                for i in range(2801):
                    stamp = rospy.Time.from_sec(1000+i*.005)
                    imu.header.stamp = stamp
                    bag.write('/test/imu', imu, stamp)
                    if i % 20 == 0:
                        message = corrupt if 6.0 <= i*.005 < 6.9 else scan
                        message.header.stamp = stamp
                        bag.write('/test/points', message, stamp)
                        if with_camera:
                            image.header.stamp=stamp+rospy.Duration.from_sec(.003)
                            bag.write('/test/images',image,image.header.stamp)
            parameters = yaml.safe_load((package/'config/avia.yaml').read_text())
            merge(parameters, yaml.safe_load((package/'config/camera_pinhole.yaml').read_text()))
            merge(parameters, yaml.safe_load((package/'config/localization.yaml').read_text()))
            parameters['common'].update(img_en=1, lidar_en=1, img_topic='/test/images',
                lid_topic='/test/points', imu_topic='/test/imu', direct_bag_path='' if live else str(bag_path), img_buffer_max=10)
            parameters['preprocess'].update(lidar_type=4, point_filter_num=1, blind=.1, filter_size_surf=.15)
            parameters['extrin_calib'].update(extrinsic_R=R_imu_lidar.ravel().tolist(), extrinsic_T=T_imu_lidar.tolist())
            parameters['localization'].update(map_path=str(map_path), metadata_path=str(map_path)+'.yaml',
                calibration_id='synthetic-calibration', sensor_timeout=1.0)
            parameters['evo']['pose_output_en'] = False
            rospy.set_param('/laserMapping', parameters)
            rospy.set_param('/use_sim_time',live)
            executable = roslib.packages.find_node('sa_livo', 'fastlivo_mapping')[0]
            with (root/'mapper.log').open('w') as log:
                process = subprocess.Popen([executable, '__name:=laserMapping'], stdout=log, stderr=subprocess.STDOUT)
                try:
                    if live:
                        import rosgraph
                        deadline=time.monotonic()+10
                        while time.monotonic()<deadline:
                            state=rosgraph.Master(rospy.get_name()).getSystemState()
                            if any(topic=='/test/points' for topic,nodes in state[1]):
                                break
                            time.sleep(.05)
                        player=subprocess.Popen(['rosbag','play','--clock','--delay=.3','--rate=1',str(bag_path)],stdout=log,stderr=subprocess.STDOUT)
                        try:
                            player.wait(timeout=35)
                            self.assertEqual(player.returncode,0)
                            time.sleep(.5)
                        finally:
                            if player.poll() is None:
                                player.send_signal(signal.SIGINT);player.wait(timeout=5)
                        process.send_signal(signal.SIGINT)
                    process.wait(timeout=65)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=10)
                    self.fail('mapper did not drain the synthetic bag\n'+(root/'mapper.log').read_text()[-7000:])
                finally:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        process.wait(timeout=10)
            time.sleep(.3)
            logs = (root/'mapper.log').read_text()
            self.assertEqual(process.returncode, 0, logs[-10000:])
            self.assertGreaterEqual(logs.count('applied recovery'), 2, logs[-12000:])
            self.assertTrue(any(s.state == LocalizationStatus.TRACKING for s in statuses), logs[-10000:])
            self.assertTrue(any(s.state in (LocalizationStatus.LOST, LocalizationStatus.WAIT_STATIONARY)
                                and s.source_stamp.to_sec() >= 1006 for s in statuses), logs[-10000:])
            self.assertGreater(len(poses), 10, logs[-10000:])
            self.assertTrue(all(p.header.frame_id == 'map' for p in poses))
            valid_stamps = {s.source_stamp.to_nsec() for s in statuses if s.valid}
            self.assertTrue(all(p.header.stamp.to_nsec() in valid_stamps for p in poses))
            last = poses[-1].pose.pose.position
            self.assertLess(np.linalg.norm(np.array([last.x, last.y, last.z])-expected), .15)
            quaternion = poses[-1].pose.pose.orientation
            q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
            self.assertAlmostEqual(np.linalg.norm(q), 1.0, places=5)
            print('synthetic pipeline: %d applied recoveries, %d valid poses' %
                  (logs.count('applied recovery'), len(poses)))
        for subscription in subscriptions:
            subscription.unregister()


if __name__ == '__main__':
    rospy.init_node('localization_pipeline_test')
    rostest.rosrun('sa_livo', 'localization_pipeline', PipelineTest)
