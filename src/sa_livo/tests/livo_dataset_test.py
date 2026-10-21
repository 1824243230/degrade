#!/usr/bin/python3
import importlib.util
import sys
from pathlib import Path
import tempfile
import unittest
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Imu

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from prepare_livo_dataset import audit, calibration


class DatasetTest(unittest.TestCase):
    def test_reject_wrong_epoch_and_missing_topics(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);bag=p/'input.bag';reference=p/'ref.csv'
            with rosbag.Bag(str(bag),'w') as output:
                for i in range(5):
                    m=Imu();m.header.stamp=rospy.Time.from_sec(1700000000+i*.1)
                    output.write('/imu/data',m,m.header.stamp)
            reference.write_text('timestamp,p_w_b_x,p_w_b_y,p_w_b_z,q_w_b_x,q_w_b_y,q_w_b_z,q_w_b_w\n'+
                ''.join('%d,0,0,0,0,0,0,1\n'%int((1500000000+i*.1)*1e9) for i in range(5)))
            result=audit(bag,['/imu/data'],reference)
            self.assertTrue(result['valid']);self.assertFalse(result['reference']['matches_sensor_epoch'])
            self.assertFalse(audit(bag,['/missing'])['valid'])

    def test_given_calibration_chain_and_rotation_correction(self):
        il,ic,cam,report=calibration(ROOT/'databag/传感器标定参数.md')
        cl=np.linalg.inv(ic)@il
        point=np.array([2.,.4,.7,1.])
        self.assertTrue(np.allclose(ic@(cl@point),il@point))
        self.assertTrue(np.allclose(il[:3,:3].T@il[:3,:3],np.eye(3),atol=1e-10))
        self.assertGreater(report[0]['correction_frobenius'],0)
        self.assertEqual(cam['model_type'],'MEI')

    def test_mei_central_ray_and_map(self):
        spec=importlib.util.spec_from_file_location('cv2','/usr/lib/python3/dist-packages/cv2.cpython-38-x86_64-linux-gnu.so')
        cv2=importlib.util.module_from_spec(spec);spec.loader.exec_module(cv2)
        K=np.array([[300.,0,160],[0,300,120],[0,0,1]])
        P=K.copy();P[0,0]=P[1,1]=120.
        x,y=cv2.omnidir.initUndistortRectifyMap(K,np.zeros(4),np.array([1.5]),np.eye(3),P,(320,240),cv2.CV_32FC1,cv2.omnidir.RECTIFY_PERSPECTIVE)
        self.assertAlmostEqual(float(x[120,160]),160,places=5)
        self.assertAlmostEqual(float(y[120,160]),120,places=5)
        self.assertTrue(np.all(np.isfinite(x)))


if __name__=='__main__':unittest.main()
