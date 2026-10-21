#!/usr/bin/python3
"""Audit provenance/time overlap, decode and rectify a confirmed LIVO dataset."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import numpy as np
import yaml
import rosbag

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def absolute(path):
    p = Path(path)
    return p if p.is_absolute() else ROOT / p


def audit(bag_path, topics, reference=None):
    stats = {}
    with rosbag.Bag(str(bag_path)) as bag:
        available = sorted(bag.get_type_and_topic_info().topics)
        missing = sorted(set(topics) - set(available))
        for topic, raw, record in bag.read_messages(topics=topics, raw=True):
            typ, data = raw[:2]
            _, sec, ns = struct.unpack_from('<III', data)
            t = sec + ns / 1e9
            s = stats.setdefault(topic, dict(count=0, first=t, last=t, backwards=0,
                                           duplicates=0, max_gap=0, types=[]))
            if s['count']:
                dt = t - s['last']
                s['backwards'] += int(dt < 0)
                s['duplicates'] += int(dt == 0)
                s['max_gap'] = max(s['max_gap'], dt)
            s['count'] += 1
            s['last'] = t
            if typ not in s['types']:
                s['types'].append(typ)
    result = dict(bag=str(bag_path), available_topics=available, missing_topics=missing,
                  streams=stats, sensor_time_source='header.stamp', reference=None)
    if reference:
        with open(reference) as f:
            rows = list(csv.DictReader(f))
        times = np.array([int(r['timestamp']) / 1e9 for r in rows])
        if len(times) < 3 or np.any(np.diff(times) <= 0):
            raise ValueError('Reference timestamps must increase strictly')
        overlap = 0.0
        if stats and not missing:
            first = max(s['first'] for s in stats.values())
            last = min(s['last'] for s in stats.values())
            overlap = max(0, min(last, times[-1]) - max(first, times[0]))
        result['reference'] = dict(path=str(reference), poses=len(rows), first=float(times[0]),
            last=float(times[-1]), duration=float(times[-1] - times[0]), overlap_seconds=overlap,
            matches_sensor_epoch=bool(overlap > 0), sha256=digest(reference))
    result['valid'] = not missing and all(s['count'] > 0 and not s['backwards'] and not s['duplicates']
                                         for s in stats.values())
    return result


def calibration(path):
    text = path.read_text().replace('<br/>', '\n')
    values = re.findall(r'data:\s*\[([^]]+)\]', text)
    if len(values) != 2:
        raise ValueError('Expected laser_to_imu and rgb_camera_to_imu matrices')
    matrices = [np.array([float(x.strip()) for x in v.split(',')]).reshape(4, 4) for v in values]
    report = []
    for T in matrices:
        if not np.all(np.isfinite(T)) or not np.allclose(T[3], [0, 0, 0, 1]):
            raise ValueError('Invalid calibration transform')
        raw = T[:3, :3].copy()
        u, _, vt = np.linalg.svd(raw)
        fix = np.eye(3); fix[2, 2] = np.linalg.det(u @ vt)
        T[:3, :3] = u @ fix @ vt
        correction = np.linalg.norm(T[:3, :3] - raw)
        if correction > .01:
            raise ValueError('Rotation correction exceeds small-rounding-error allowance')
        report.append(dict(original=raw.tolist(), used=T[:3, :3].tolist(),
                           correction_frobenius=float(correction)))
    cam = yaml.safe_load(text[text.index('rgb_camera:'):])['rgb_camera']
    if cam['model_type'] != 'MEI':
        raise ValueError('This adapter requires a MEI calibration')
    return matrices[0], matrices[1], cam, report


def prepare(profile_path, output, audit_only=False):
    profile = yaml.safe_load(profile_path.read_text())
    source = absolute(profile['bag'])
    reference = absolute(profile['reference']) if profile.get('reference') else None
    output.mkdir(parents=True, exist_ok=False)
    topics = [profile[k] for k in ('lidar_topic', 'imu_topic', 'image_topic')]
    report = audit(source, topics, reference)
    (output / 'input_audit.json').write_text(json.dumps(report, indent=2) + '\n')
    if audit_only:
        return report
    if not report['valid']:
        raise ValueError('Missing/nonmonotonic sensor streams; inspect input_audit.json')
    if not profile.get('calibration_confirmed') or not profile.get('calibration_source'):
        raise ValueError('Calibration-topic association must be confirmed with its source')
    if reference and (not profile.get('reference_confirmed') or
                      not report['reference']['matches_sensor_epoch']):
        raise ValueError('Reference does not match sensor time; no timestamp shifting is allowed')
    cal_path = absolute(profile['calibration'])
    T_IL, T_IC, camera, rotations = calibration(cal_path)
    T_CL = np.linalg.inv(T_IC) @ T_IL
    pr, di = camera['projection_parameters'], camera['distortion_parameters']
    xi = float(camera['mirror_parameters']['xi'])
    K = np.array([[pr['gamma1'], 0, pr['u0']], [0, pr['gamma2'], pr['v0']], [0, 0, 1.]])
    # Preserve the MEI model's central angular magnification in the virtual camera.
    virtual = K.copy(); virtual[0, 0] /= 1 + xi; virtual[1, 1] /= 1 + xi
    width, height = camera['image_width'], camera['image_height']
    kind = profile['lidar_kind']
    base_config = 'avia.yaml' if kind in ('avia', 'mid360') else 'long_corridor.yaml'
    params = yaml.safe_load((ROOT / 'src/sa_livo/config' / base_config).read_text())
    params['common'].update(img_en=1, lidar_en=1, img_topic='/camera_1/image_rect',
        imu_topic='/imu/data', img_compressed_en=False, img_rate_hz=10.0, img_buffer_max=20,
        image_valid_mask=str(output / 'valid_mask.png'))
    params['extrin_calib'] = dict(extrinsic_T=T_IL[:3, 3].tolist(), extrinsic_R=T_IL[:3, :3].ravel().tolist(),
        Rcl=T_CL[:3, :3].ravel().tolist(), Pcl=T_CL[:3, 3].tolist())
    params['time_offset'] = dict(imu_time_offset=profile.get('imu_time_offset', 0.),
                               img_time_offset=profile.get('img_time_offset', 0.), exposure_time_init=0.)
    params['imu']['gravity_norm'] = profile.get('gravity_norm', 9.81)
    if kind in ('avia', 'mid360'):
        params['imu'].update(stationary_init=True, stationary_duration=1.,
                             stationary_gyro_max=.05,
                             stationary_acc_std=float(profile.get('stationary_acc_std', .10)))
    params['sa_vio'] = dict(dual_event_en=False, window_size=5, min_obs=30, max_obs_per_frame=100,
        patch_spacing=8, depth_min=.5, theta_max_deg=80., parallax_min_px=.5,
        abs_res_max=15., chi2_thres=6.635, sigma_px=1., info_scale=1.)
    params['pcd_save']['pcd_save_en'] = False
    params['evo'].update(seq_name=profile.get('seq_name', 'long_corridor_livo'), pose_output_en=True)
    preparation = dict(calibration_confirmed=True, lidar_topic=profile['lidar_topic'],
        imu_topic=profile['imu_topic'], image_topic=profile['image_topic'],
        imu_acc_scale=profile['imu_acc_scale'], image_rate_hz=profile['image_rate_hz'],
        mei=dict(width=width, height=height, K=K.ravel().tolist(),
                 D=[di[k] for k in ('k1', 'k2', 'p1', 'p2')], xi=[xi]),
        virtual_camera=dict(width=width, height=height, K=virtual.ravel().tolist()))
    identity = dict(profile_sha256=digest(profile_path), input_sha256=digest(source),
        calibration_sha256=digest(cal_path), reference_sha256=digest(reference) if reference else None,
        rotations=rotations, T_IL=T_IL.tolist(), T_IC=T_IC.tolist(), T_CL=T_CL.tolist(),
        converter_sha256=digest(ROOT / 'devel/lib/sa_livo/rectify_mei_bag'),
        decoder_sha256=digest(ROOT / 'devel/lib/sa_livo/decode_velodyne_bag'),
        reference_note=profile.get('reference_note', ''), original_bag=str(source),
        source_profile=str(profile_path.resolve()))
    if kind == 'velodyne_packets':
        decoded = output / 'decoded_livo.bag'
        decoder = ROOT / 'devel/lib/sa_livo/decode_velodyne_bag'
        with open(output / 'decode.log', 'w') as log:
            subprocess.run([str(decoder), str(source), str(decoded),
                '/opt/ros/noetic/share/velodyne_pointcloud/params/VLP16db.yaml', '--images'],
                check=True, stdout=log, stderr=subprocess.STDOUT)
        preparation.update(lidar_topic='/velodyne_points', imu_topic='/imu/data')
        source = decoded
    elif kind in ('avia', 'mid360'):
        params['common']['lid_topic'] = '/livox/lidar'
        params['preprocess'].update(lidar_type=1, scan_line=6 if kind == 'avia' else 4)
    else:
        raise ValueError('Unsupported lidar_kind: ' + kind)
    (output / 'preparation.yaml').write_text(yaml.safe_dump(preparation, sort_keys=False))
    (output / 'parameters.yaml').write_text(yaml.safe_dump(params, sort_keys=False))
    pinhole = dict(model='Pinhole', width=width, height=height, scale=1., fx=float(virtual[0, 0]),
                   fy=float(virtual[1, 1]), cx=float(virtual[0, 2]), cy=float(virtual[1, 2]))
    (output / 'camera.yaml').write_text(yaml.safe_dump(dict(camera=pinhole), sort_keys=False))
    (output / 'input_identity.json').write_text(json.dumps(identity, indent=2) + '\n')
    with open(output / 'rectify.log', 'w') as log:
        subprocess.run([str(ROOT / 'devel/lib/sa_livo/rectify_mei_bag'), str(source),
            str(output / 'normalized_livo.bag'), str(output / 'preparation.yaml')],
            check=True, stdout=log, stderr=subprocess.STDOUT)
    # Keep the decoded intermediate for inspection; subsequent experiments share normalized input.
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=ROOT / 'src/sa_livo/config/long_corridor_livo_profile.yaml')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--audit-only', action='store_true')
    args = parser.parse_args()
    try:
        result = prepare(args.profile.resolve(), args.output.resolve(), args.audit_only)
        print(json.dumps(result, indent=2))
    except Exception as error:
        print(str(error), file=sys.stderr)
        sys.exit(1)
