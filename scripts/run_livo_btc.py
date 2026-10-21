#!/usr/bin/python3
"""Run paired LIVO experiments and preserve independent outputs and provenance."""
import argparse
import json
import os
from pathlib import Path
import platform
import re
import shutil
import socket
import subprocess
import sys
import time
import numpy as np
from prepare_livo_dataset import ROOT, digest, prepare, absolute
from evaluate_long_corridor import evaluate, summary
import yaml


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def verify_prepared(profile_path, prepared):
    identity = json.loads((prepared / 'input_identity.json').read_text())
    profile = yaml.safe_load(profile_path.read_text())
    if identity['profile_sha256'] != digest(profile_path):
        raise ValueError('Prepared profile differs; use a new output root')
    checks = [('calibration_sha256', absolute(profile['calibration'])),
              ('reference_sha256', absolute(profile['reference'])),
              ('converter_sha256', ROOT / 'devel/lib/sa_livo/rectify_mei_bag'),
              ('decoder_sha256', ROOT / 'devel/lib/sa_livo/decode_velodyne_bag')]
    for key, path in checks:
        if digest(path) != identity[key]:
            raise ValueError('Prepared input identity differs: ' + key)
    # Hash the recording once per experiment batch, not on every repetition.
    if digest(absolute(profile['bag'])) != identity['input_sha256']:
        raise ValueError('Original recording differs')
    return profile


def replay(prepared, directory, enabled):
    directory.mkdir(parents=True, exist_ok=False)
    port = free_port()
    env = os.environ.copy()
    env['ROS_MASTER_URI'] = 'http://127.0.0.1:' + str(port)
    args = ['roslaunch', '--port', str(port), 'sa_livo', 'mapping_livo_loop.launch',
        'params_file:=' + str(prepared / 'parameters.yaml'),
        'camera_params_file:=' + str(prepared / 'camera.yaml'),
        'direct_bag_path:=' + str(prepared / 'normalized_livo.bag'),
        'output_dir:=' + str(directory), 'loop_enabled:=' + str(enabled).lower()]
    (directory / 'command.json').write_text(json.dumps(args, indent=2) + '\n')
    for name in ('parameters.yaml', 'camera.yaml', 'input_identity.json'):
        shutil.copy2(prepared / name, directory / name)
    shutil.copy2(ROOT / 'src/sa_livo/config/btc_loop.yaml', directory / 'loop_parameters.yaml')
    shutil.copy2(ROOT / 'src/btc_descriptor/config/config_indoor.yaml', directory / 'btc_parameters.yaml')
    with open(directory / 'replay.log', 'w') as log:
        process = subprocess.run(['/usr/bin/time', '-f',
            '{"wall_seconds":%e,"user_seconds":%U,"system_seconds":%S,"max_rss_kib":%M,"exit_code":%x}',
            '-o', str(directory / 'runtime.json')] + args, env=env,
            stdout=log, stderr=subprocess.STDOUT, timeout=1800)
    text = (directory / 'replay.log').read_text(errors='replace')
    if process.returncode or 'SA-LIVO failed:' in text or not re.search(r'finished: imu=\d+ lidar=\d+ img=[1-9]\d*', text):
        raise RuntimeError('LIVO replay failed; inspect ' + str(directory / 'replay.log'))
    q = np.atleast_1d(np.genfromtxt(directory / 'livo_quality.csv', delimiter=',', names=True))
    if len(q) < 20:
        raise RuntimeError('Too few LIVO updates')
    return q


def summarize_runs(root, reference, repetitions):
    results = []
    for enabled, group in ((False, 'livo'), (True, 'livo_btc')):
        for rep in range(1, repetitions + 1):
            directory = root / group / ('run_%02d' % rep)
            quality = directory / 'livo_quality.csv'
            raw = evaluate(directory / 'raw_trajectory.tum', reference, quality, directory / 'raw_evaluation', 'LIVO raw')
            optimized = evaluate(directory / 'optimized_trajectory.tum', reference, quality,
                directory / 'optimized_evaluation', 'LIVO + BTC optimized') if enabled else raw
            runtime = json.loads((directory / 'runtime.json').read_text())
            item = dict(group=group, repetition=rep, raw=raw, optimized=optimized, runtime=runtime)
            if enabled:
                item['backend'] = json.loads((directory / 'backend_summary.json').read_text())
                times = np.atleast_1d(np.genfromtxt(directory / 'backend_timing.csv', delimiter=',', names=True))
                item['backend_times_ms'] = {k: summary(times[k]) for k in
                    ('extraction_ms', 'query_ms', 'verification_ms', 'optimization_ms')}
                a = np.atleast_2d(np.loadtxt(root / 'livo' / ('run_%02d' % rep) / 'raw_trajectory.tum'))
                b = np.atleast_2d(np.loadtxt(directory / 'raw_trajectory.tum'))
                item['frontend_matches_baseline'] = bool(a.shape == b.shape and np.allclose(a, b, rtol=0, atol=1e-7))
            results.append(item)
    # Intervals are defined only from the baseline's information diagnostics.
    q = np.atleast_1d(np.genfromtxt(root / 'livo/run_01/livo_quality.csv', delimiter=',', names=True))
    eigen = np.column_stack([q['lidar_scaled_lambda' + str(k)] for k in range(6)])
    weak = (eigen[:, 0] / np.maximum(eigen[:, 5], 1e-12) < 1e-4) | (q['map_blocked'] != 0)
    intervals = []
    for i in range(len(q)):
        if weak[i] and (i == 0 or not weak[i-1]): start = float(q['timestamp'][i])
        if weak[i] and (i == len(q)-1 or not weak[i+1]):
            intervals.append([start, float(q['timestamp'][i])])
    (root / 'degenerate_intervals.json').write_text(json.dumps(dict(
        rule='baseline scaled LiDAR lambda_min/lambda_max < 1e-4 or map blocked; rotation length 2 m',
        diagnostic_only=True, intervals=intervals), indent=2) + '\n')
    for item in results:
        directory=root/item['group']/('run_%02d'%item['repetition'])
        for name in ('raw_evaluation','optimized_evaluation'):
            if not (directory/name/'errors.csv').exists(): continue
            errors=np.atleast_2d(np.loadtxt(directory/name/'errors.csv',delimiter=',',skiprows=1))
            rpe=np.atleast_2d(np.loadtxt(directory/name/'rpe_1s.csv',delimiter=',',skiprows=1))
            mask=np.zeros(len(errors),dtype=bool);rpe_mask=np.zeros(len(rpe),dtype=bool)
            for start,end in intervals:
                mask |= (errors[:,0]>=start)&(errors[:,0]<=end)
                rpe_mask |= (rpe[:,0]>=start)&(rpe[:,1]<=end)
            item.setdefault('segments',{})[name]=dict(degenerate=dict(
                frames=int(mask.sum()),ate_translation_m=summary(errors[mask,1]),
                rpe_1s_translation_m=summary(rpe[rpe_mask,2])))
            first_loop=None
            if item['group']=='livo_btc':
                import csv
                accepted=[float(row['timestamp']) for row in csv.DictReader(open(directory/'loops.csv'))
                          if row['accepted']=='1' and row['reason']=='accepted']
                if accepted: first_loop=min(accepted)
            for key,selected in (('before_first_loop',np.ones(len(errors),bool) if first_loop is None else errors[:,0]<first_loop),
                                 ('after_first_loop',np.zeros(len(errors),bool) if first_loop is None else errors[:,0]>=first_loop)):
                item['segments'][name][key]=dict(frames=int(selected.sum()),ate_translation_m=summary(errors[selected,1]))
            item['first_loop_anchor_time']=first_loop
    from evaluate_livo_map import evaluate_revisits
    overall_map=evaluate_revisits(root/'livo_btc/run_01',reference)
    (root/'revisit_consistency.json').write_text(json.dumps(overall_map,indent=2,allow_nan=False)+'\n')
    overall = dict(results=results, reference=str(reference), reference_used_only_for_evaluation=True,
        map_revisit_consistency=overall_map,
        loop_review='Accepted edges require independent visual/reference review; no recall claim without full labels.')
    (root / 'comparison.json').write_text(json.dumps(overall, indent=2, allow_nan=False) + '\n')
    plot_comparison(root, results, reference)
    plot_diagnostics(root)
    return overall


def plot_comparison(root, results, reference):
    os.environ.setdefault('MPLCONFIGDIR', str(root / '.matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    ref=np.atleast_2d(np.loadtxt(reference,delimiter=',',skiprows=1))
    axes[0].plot(ref[:,1],ref[:,2],color='black',linestyle='--',linewidth=1,label='Provided reference')
    for group, filename, title in (('livo', 'raw_evaluation', 'LIVO'),
            ('livo_btc', 'optimized_evaluation', 'LIVO + BTC')):
        path = root / group / 'run_01' / filename
        p = np.atleast_2d(np.loadtxt(path / 'aligned_trajectory.tum'))
        errors = np.atleast_2d(np.loadtxt(path / 'errors.csv', delimiter=',', skiprows=1))
        axes[0].plot(p[:, 1], p[:, 2], label=title)
        axes[1].plot(errors[:, 0] - errors[0, 0], errors[:, 1], label=title)
    axes[0].axis('equal'); axes[0].set(xlabel='x [m]', ylabel='y [m]', title='SE(3) aligned trajectories')
    axes[1].set(xlabel='Time from first output [s]', ylabel='Position error [m]')
    for ax in axes: ax.grid(alpha=.3); ax.legend()
    fig.tight_layout(); fig.savefig(root / 'comparison.png', dpi=160); plt.close(fig)


def plot_diagnostics(root):
    import matplotlib.pyplot as plt
    q=np.atleast_1d(np.genfromtxt(root/'livo/run_01/livo_quality.csv',delimiter=',',names=True))
    t=q['timestamp']-q['timestamp'][0]
    ratio=q['lidar_scaled_lambda0']/np.maximum(q['lidar_scaled_lambda5'],1e-12)
    gates=np.column_stack([q['gate'+str(k)] for k in range(6)])
    fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
    axes[0].semilogy(t,np.maximum(ratio,1e-8),label='Scaled LiDAR eigenvalue ratio')
    axes[0].axhline(1e-4,color='r',linestyle='--',label='Fixed diagnostic threshold')
    axes[0].set(ylabel='min / max eigenvalue');axes[0].legend()
    axes[1].plot(t,q['q'],label='Visual quality q');axes[1].plot(t,np.min(gates,axis=1),label='Minimum SAIF weight',alpha=.7)
    axes[1].set(ylabel='Weight');axes[1].legend()
    axes[2].plot(t,q['visual_valid'],label='Valid visual observations')
    axes[2].set(ylabel='Observations',xlabel='Time from first output [s]');axes[2].legend()
    for ax in axes:ax.grid(alpha=.3)
    fig.tight_layout();fig.savefig(root/'diagnostics.png',dpi=160);plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=ROOT / 'src/sa_livo/config/long_corridor_livo_profile.yaml')
    parser.add_argument('--output', type=Path, default=ROOT / 'validation/long_corridor_livo_btc')
    parser.add_argument('--repetitions', type=int, default=3)
    parser.add_argument('--evaluate-only', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.repetitions <= 10: parser.error('repetitions must be between 1 and 10')
    root = args.output.resolve(); root.mkdir(parents=True, exist_ok=True)
    prepared = root / 'input'
    if not prepared.exists(): prepare(args.profile.resolve(), prepared)
    profile = verify_prepared(args.profile.resolve(), prepared)
    if not args.evaluate_only:
        environment = dict(platform=platform.platform(), machine=platform.machine(), cpu_count=os.cpu_count(),
            cpuinfo=Path('/proc/cpuinfo').read_text().split('\n\n')[0],
            executable_sha256=digest(ROOT / 'devel/lib/sa_livo/fastlivo_mapping'),
            loop_library_sha256=digest(ROOT / 'devel/lib/libloop_closure.so'))
        (root / 'environment.json').write_text(json.dumps(environment, indent=2) + '\n')
        for rep in range(1, args.repetitions + 1):
            for enabled, group in ((False, 'livo'), (True, 'livo_btc')):
                directory = root / group / ('run_%02d' % rep)
                if directory.exists():
                    raise ValueError('Run exists; use a new output or --evaluate-only: ' + str(directory))
                print('Running', group, rep, flush=True)
                replay(prepared, directory, enabled)
    summarize_runs(root, absolute(profile['reference']), args.repetitions)
    print('Comparison written to', root / 'comparison.json', flush=True)


if __name__ == '__main__':
    try: main()
    except Exception as error:
        print(str(error), file=sys.stderr); sys.exit(1)
