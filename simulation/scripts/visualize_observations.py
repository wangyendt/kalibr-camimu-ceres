#!/usr/bin/env python3
"""Visualize a simulated cam-IMU dataset: 3D trajectory + per-frame corner observations.

Outputs (under <dataset>/viz/):
  - trajectory_3d.png      3D body trajectory + orientation + target corner cloud
  - corners_per_frame.png  observed corners vs frame (coverage over time)
  - corners_<cam>.mp4       per-frame corner observation playback (OpenCV)
  - corner_montage.png      a contact sheet of sampled frames

Use --show to also play the OpenCV window live.
"""

import argparse
import collections
import csv
import pathlib

import cv2
import numpy as np
import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def load_camchain_resolution(dataset: pathlib.Path, cam: str):
    chain = yaml.safe_load((dataset / "camchain.yaml").read_text())
    res = chain[cam]["resolution"]
    return int(res[0]), int(res[1])


def load_corners(path: pathlib.Path):
    """Group corner rows by frame timestamp. Returns ordered dict ts_ns -> list of dict."""
    frames = collections.OrderedDict()
    with path.open() as f:
        for row in csv.DictReader(f):
            ts = int(row["timestamp_ns"])
            frames.setdefault(ts, []).append({
                "corner_id": int(row["corner_id"]),
                "u": float(row["u_px"]),
                "v": float(row["v_px"]),
                "wx": float(row["target_x_m"]),
                "wy": float(row["target_y_m"]),
                "wz": float(row["target_z_m"]),
                "target_index": int(row["target_index"]),
            })
    return frames


def load_trajectory(path: pathlib.Path):
    ts, xyz, quat = [], [], []
    with path.open() as f:
        for row in csv.DictReader(f):
            ts.append(float(row["timestamp_s"]))
            xyz.append([float(row["tx"]), float(row["ty"]), float(row["tz"])])
            quat.append([float(row["qw"]), float(row["qx"]),
                         float(row["qy"]), float(row["qz"])])
    return np.array(ts), np.array(xyz), np.array(quat)


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def target_color(target_index):
    # BGR for OpenCV
    return (80, 220, 80) if target_index == 0 else (60, 220, 230)


def report_coverage(frames, expected_fps, duration_s, total_corners):
    counts = np.array([len(v) for v in frames.values()])
    ts = np.array(sorted(frames.keys())) * 1e-9
    expected_frames = int(round(duration_s * expected_fps)) + 1
    print("=" * 60)
    print(f"frames in corner file : {len(frames)}")
    print(f"expected frames (@{expected_fps:g}Hz x {duration_s:g}s) : {expected_frames}")
    print(f"empty/missing frames  : {expected_frames - len(frames)}")
    print(f"corners/frame         : min {counts.min()}  max {counts.max()}  "
          f"mean {counts.mean():.1f}  (target total {total_corners})")
    if len(ts) > 1:
        dt = np.diff(ts)
        print(f"frame dt (s)          : median {np.median(dt):.4f}  "
              f"max {dt.max():.4f}  -> all frames observed: "
              f"{'YES' if abs(dt.max() - np.median(dt)) < 1e-6 else 'NO (gaps!)'}")
    print("=" * 60)
    return counts, ts


def plot_trajectory_3d(traj_xyz, traj_quat, frames, out_path):
    # World corner cloud from the corner observations (world == target0 frame).
    world = {}
    for rows in frames.values():
        for r in rows:
            world[r["corner_id"]] = (r["wx"], r["wy"], r["wz"], r["target_index"])
    pts = np.array([[v[0], v[1], v[2]] for v in world.values()])
    tix = np.array([v[3] for v in world.values()])

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(traj_xyz[:, 0], traj_xyz[:, 1], traj_xyz[:, 2],
            "-", color="tab:blue", lw=1.2, label="body trajectory")
    ax.scatter(traj_xyz[0, 0], traj_xyz[0, 1], traj_xyz[0, 2],
               c="green", s=40, label="start")
    ax.scatter(traj_xyz[-1, 0], traj_xyz[-1, 1], traj_xyz[-1, 2],
               c="red", s=40, label="end")
    for ti, color in [(0, "tab:green"), (1, "tab:orange")]:
        m = tix == ti
        if m.any():
            ax.scatter(pts[m, 0], pts[m, 1], pts[m, 2], c=color, s=6,
                       label=f"target{ti} corners")
    # Orientation: camera viewing axis (body +z) at sampled poses.
    step = max(1, len(traj_xyz) // 25)
    for i in range(0, len(traj_xyz), step):
        R = quat_to_R(traj_quat[i])
        d = R[:, 2] * 0.12
        p = traj_xyz[i]
        ax.quiver(p[0], p[1], p[2], d[0], d[1], d[2],
                  color="gray", lw=0.8, arrow_length_ratio=0.3)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("Simulated body trajectory (world = target0) + target corners")
    ax.legend(loc="upper left", fontsize=8)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    print(f"saved {out_path}")


def plot_corners_per_frame(counts, ts, total_corners, out_path):
    fig, ax = plt.subplots(figsize=(10, 3.2))
    ax.plot(ts, counts, "-o", ms=2, lw=1, color="tab:blue")
    ax.axhline(total_corners, color="gray", ls="--", lw=1,
               label=f"target total ({total_corners})")
    ax.set_xlabel("time (s)"); ax.set_ylabel("corners observed")
    ax.set_title("Observed corners per frame")
    ax.set_ylim(0, total_corners * 1.05)
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(out_path, dpi=130); plt.close(fig)
    print(f"saved {out_path}")


def draw_frame(rows, w, h, frame_idx, n_frames, ts_s, total_corners):
    img = np.full((h, w, 3), 28, dtype=np.uint8)
    # border
    cv2.rectangle(img, (0, 0), (w - 1, h - 1), (70, 70, 70), 1)
    # group corners of each apriltag (4 consecutive corner ids) into a quad
    by_tag = collections.defaultdict(dict)
    for r in rows:
        by_tag[r["corner_id"] // 4][r["corner_id"] % 4] = r
    for tag, cs in by_tag.items():
        if len(cs) == 4:
            quad = np.array([[int(cs[k]["u"]), int(cs[k]["v"])] for k in range(4)],
                            dtype=np.int32)
            cv2.polylines(img, [quad], True, (90, 90, 90), 1, cv2.LINE_AA)
    for r in rows:
        u, v = int(round(r["u"])), int(round(r["v"]))
        cv2.circle(img, (u, v), 2, target_color(r["target_index"]), -1, cv2.LINE_AA)
    # HUD
    cv2.putText(img, f"frame {frame_idx + 1}/{n_frames}  t={ts_s:6.3f}s  "
                f"corners={len(rows)}/{total_corners}",
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1, cv2.LINE_AA)
    return img


def main():
    repo = pathlib.Path(__file__).resolve().parents[2]
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", type=pathlib.Path,
                    default=repo / "simulation/generated/one_cam_one_imu")
    ap.add_argument("--cam", default="cam0")
    ap.add_argument("--fps", type=float, default=20.0,
                    help="expected camera fps for coverage check")
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--video-fps", type=float, default=15.0)
    ap.add_argument("--montage", type=int, default=12,
                    help="number of frames in the contact sheet")
    ap.add_argument("--show", action="store_true",
                    help="also play an OpenCV window live")
    args = ap.parse_args()

    dataset = args.dataset.expanduser().resolve()
    cam = args.cam
    w, h = load_camchain_resolution(dataset, cam)
    frames = load_corners(dataset / f"{cam}_corners.csv")
    traj_ts, traj_xyz, traj_quat = load_trajectory(dataset / "trajectory_keyframes.csv")
    total_corners = sum(1 for _ in (dataset / "target_points.csv").open()) - 1

    counts, ts = report_coverage(frames, args.fps, args.duration, total_corners)

    out = dataset / "viz"
    out.mkdir(exist_ok=True)
    plot_trajectory_3d(traj_xyz, traj_quat, frames, out / "trajectory_3d.png")
    plot_corners_per_frame(counts, ts, total_corners, out / "corners_per_frame.png")

    # Per-frame OpenCV video + montage
    items = list(frames.items())
    video_path = out / f"corners_{cam}.mp4"
    vw = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"),
                         args.video_fps, (w, h))
    montage_idx = set(np.linspace(0, len(items) - 1, args.montage).round().astype(int))
    montage_imgs = []
    for i, (ts_ns, rows) in enumerate(items):
        img = draw_frame(rows, w, h, i, len(items), ts_ns * 1e-9, total_corners)
        vw.write(img)
        if i in montage_idx:
            montage_imgs.append(img.copy())
        if args.show:
            cv2.imshow("corners", img)
            if cv2.waitKey(int(1000 / args.video_fps)) == 27:  # ESC
                break
    vw.release()
    if args.show:
        cv2.destroyAllWindows()
    print(f"saved {video_path}")

    # contact sheet
    if montage_imgs:
        cols = 4
        rows_n = int(np.ceil(len(montage_imgs) / cols))
        sheet = np.full((rows_n * h, cols * w, 3), 0, dtype=np.uint8)
        for k, im in enumerate(montage_imgs):
            r, c = divmod(k, cols)
            sheet[r * h:(r + 1) * h, c * w:(c + 1) * w] = im
        sheet_small = cv2.resize(sheet, (cols * w // 2, rows_n * h // 2))
        cv2.imwrite(str(out / "corner_montage.png"), sheet_small)
        print(f"saved {out / 'corner_montage.png'}")


if __name__ == "__main__":
    main()
