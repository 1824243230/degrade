#!/usr/bin/python3
"""Evaluate IMU-frame TUM output against the supplied nanosecond CSV reference.

Uses time interpolation and rigid SE(3) alignment only (no fitted scale).
Reference calibration/accuracy must be established separately.
"""
import argparse
import json
import os
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def summary(values):
    a = np.asarray(values)
    a = a[np.isfinite(a)]
    if not a.size:
        return None
    return dict(rmse=float(np.sqrt(np.mean(a*a))), mean=float(np.mean(a)),
                median=float(np.median(a)), p95=float(np.percentile(a, 95)),
                max=float(np.max(a)))


def rigid_alignment(source, target):
    u, _, vt = np.linalg.svd((source-source.mean(0)).T @ (target-target.mean(0)))
    fix = np.eye(3)
    fix[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ fix @ u.T
    return rotation, target.mean(0)-rotation @ source.mean(0)


def evaluate(trajectory, reference, quality, output):
    est = np.atleast_2d(np.loadtxt(trajectory))
    ref = np.atleast_2d(np.loadtxt(reference, delimiter=",", skiprows=1))
    if est.shape[1] != 8 or ref.shape[1] != 8 or len(est) < 3 or len(ref) < 3:
        raise ValueError("Expected nonempty TUM trajectory and 8-column reference CSV")
    if not np.all(np.isfinite(est)) or not np.all(np.isfinite(ref)):
        raise ValueError("Nonfinite trajectory/reference")
    ref[:, 0] /= 1e9
    if np.any(np.diff(est[:, 0]) <= 0) or np.any(np.diff(ref[:, 0]) <= 0):
        raise ValueError("Trajectory timestamps must increase strictly")
    # Interpolation only across short reference intervals; no extrapolation.
    idx = np.searchsorted(ref[:, 0], est[:, 0], side="right")
    mask = (est[:, 0] >= ref[0, 0]) & (est[:, 0] <= ref[-1, 0])
    interior = np.clip(idx, 1, len(ref)-1)
    mask &= (ref[interior, 0]-ref[interior-1, 0]) <= 0.3
    e = est[mask]
    if len(e) < 3:
        raise ValueError("No sufficiently overlapping sensor/reference timestamps")
    t0 = ref[0, 0]
    tr, te = ref[:, 0]-t0, e[:, 0]-t0
    p_ref = np.column_stack([np.interp(te, tr, ref[:, j]) for j in range(1, 4)])
    r_ref = Slerp(tr, Rotation.from_quat(ref[:, 4:8]))(te)
    r_est = Rotation.from_quat(e[:, 4:8])
    rot, trans = rigid_alignment(e[:, 1:4], p_ref)
    p_aligned = e[:, 1:4] @ rot.T + trans
    aligned_rotation = Rotation.from_matrix(rot) * r_est
    errors = np.linalg.norm(p_aligned-p_ref, axis=1)
    angles = np.rad2deg((r_ref.inv()*aligned_rotation).magnitude())

    # Relative pose error at exactly one second, with estimated pose interpolation.
    valid = te+1.0 <= te[-1]
    ta, tb = te[valid], te[valid]+1.0
    pa, pb = e[valid, 1:4], np.column_stack([np.interp(tb, te, e[:, j]) for j in range(1, 4)])
    ra, rb = r_est[valid], Slerp(te, r_est)(tb)
    ga, gb = p_ref[valid], np.column_stack([np.interp(tb, tr, ref[:, j]) for j in range(1, 4)])
    qa, qb = r_ref[valid], Slerp(tr, Rotation.from_quat(ref[:, 4:8]))(tb)
    delta_est, delta_ref = ra.inv().apply(pb-pa), qa.inv().apply(gb-ga)
    rpe_t = np.linalg.norm(delta_est-delta_ref, axis=1)
    rpe_r = np.rad2deg(((qa.inv()*qb).inv()*(ra.inv()*rb)).magnitude())

    output.mkdir(parents=True, exist_ok=True)
    metrics = {
        "alignment": "SE3 rigid least squares over matched positions, scale fixed at 1",
        "reference_note": "Supplied ground_truth_path.csv; accuracy and body-frame calibration not independently verified",
        "estimated_poses": len(est), "associated_poses": len(e),
        "estimated_time_range_s": est[[0, -1], 0].tolist(),
        "reference_time_range_s": ref[[0, -1], 0].tolist(),
        "reference_coverage": float((e[-1, 0]-e[0, 0])/(ref[-1, 0]-ref[0, 0])),
        "estimated_path_length_m": float(np.linalg.norm(np.diff(est[:, 1:4], axis=0), axis=1).sum()),
        "reference_path_length_m": float(np.linalg.norm(np.diff(ref[:, 1:4], axis=0), axis=1).sum()),
        "ate_translation_m": summary(errors), "ate_rotation_deg": summary(angles),
        "rpe_1s_translation_m": summary(rpe_t), "rpe_1s_rotation_deg": summary(rpe_r),
        "alignment_rotation": rot.tolist(), "alignment_translation_m": trans.tolist(),
        "quaternion_norm_max_error": float(np.max(np.abs(np.linalg.norm(est[:, 4:8], axis=1)-1))),
    }
    if quality:
        q = np.atleast_1d(np.genfromtxt(quality, delimiter=",", names=True))
        metrics["quality"] = {
            "updates": len(q), "nonfinite_state_updates": int(np.sum(q["finite"] == 0)),
            "zero_match_updates": int(np.sum(q["matches"] == 0)),
            "degenerate_updates": int(np.sum(q["degenerate"] != 0)),
            "map_blocked_updates": int(np.sum(q["map_blocked"] != 0)),
            "matches_median": float(np.median(q["matches"])),
            "lio_compute_ms": summary(q["lio_ms"]),
            "raw_residual_rms_m": summary(q["raw_rms"]),
        }
    (output/"metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False)+"\n")
    np.savetxt(output/"aligned_trajectory.tum", np.column_stack([e[:, 0], p_aligned, aligned_rotation.as_quat()]), fmt="%.9f")
    np.savetxt(output/"errors.csv", np.column_stack([e[:, 0], errors, angles]), delimiter=",",
               header="timestamp,translation_error_m,rotation_error_deg", comments="")

    os.environ.setdefault("MPLCONFIGDIR", str(output/".matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    axes[0].plot(p_ref[:, 0], p_ref[:, 1], label="Provided reference", linewidth=1.5)
    axes[0].plot(p_aligned[:, 0], p_aligned[:, 1], label="SA-LIVO (LiDAR + IMU)", linewidth=1)
    axes[0].set(xlabel="x [m]", ylabel="y [m]", title="Trajectory (SE3 aligned)")
    axes[0].axis("equal"); axes[0].legend(fontsize=8)
    axes[1].plot(te, errors)
    axes[1].set(xlabel="Time from reference start [s]", ylabel="Position error [m]", title="Absolute trajectory error")
    axes[2].plot(te, p_ref[:, 2], label="Provided reference")
    axes[2].plot(te, p_aligned[:, 2], label="SA-LIVO")
    axes[2].set(xlabel="Time from reference start [s]", ylabel="z [m]", title="Height (SE3 aligned)")
    axes[2].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=.3)
    fig.tight_layout(); fig.savefig(output/"trajectory_comparison.png", dpi=160)
    plt.close(fig)
    print(json.dumps(metrics, indent=2))
    return metrics


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("reference", type=Path)
    parser.add_argument("--quality", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    evaluate(args.trajectory, args.reference, args.quality, args.output)
