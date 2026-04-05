import os
import numpy as np
import scipy.io as sio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RESULTS_DIR = "results"

# Prefer the MATLAB-v7 file if present, otherwise try the standard one
if os.path.exists(os.path.join(RESULTS_DIR, "evaluation_results_v7.mat")):
    EVAL_FILE = os.path.join(RESULTS_DIR, "evaluation_results_v7.mat")
else:
    EVAL_FILE = os.path.join(RESULTS_DIR, "evaluation_results.mat")

if not os.path.exists(EVAL_FILE):
    raise FileNotFoundError(f"Missing evaluation file: {EVAL_FILE}")

mat = sio.loadmat(EVAL_FILE, squeeze_me=True, struct_as_record=False)

traj_eval = mat["traj_eval"]
map_eval = mat["map_eval"]
gt_pos = np.array(mat["gt_pos"], dtype=float)
est_pos = np.array(mat["est_pos"], dtype=float)

os.makedirs(RESULTS_DIR, exist_ok=True)

def arr(x):
    return np.array(x)

# ---------- Basic masks ----------
valid_gt = np.all(np.isfinite(gt_pos), axis=1)
valid_est = np.all(np.isfinite(est_pos), axis=1)
valid_both = valid_gt & valid_est

# ---------- 1) 3D trajectory ----------
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection="3d")
if np.any(valid_gt):
    ax.plot(gt_pos[valid_gt, 0], gt_pos[valid_gt, 1], gt_pos[valid_gt, 2], label="GT")
if np.any(valid_est):
    ax.plot(est_pos[valid_est, 0], est_pos[valid_est, 1], est_pos[valid_est, 2], label="Estimated")
ax.set_title("3D camera trajectory")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.legend()
plt.tight_layout()
plt.savefig(os.path.join(RESULTS_DIR, "trajectory_3d.png"), dpi=220)
plt.close(fig)

# ---------- 2) XY trajectory ----------
plt.figure(figsize=(7, 5))
if np.any(valid_gt):
    plt.plot(gt_pos[valid_gt, 0], gt_pos[valid_gt, 1], label="GT")
if np.any(valid_est):
    plt.plot(est_pos[valid_est, 0], est_pos[valid_est, 1], label="Estimated")
plt.title("Camera trajectory in XY")
plt.xlabel("x")
plt.ylabel("y")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(RESULTS_DIR, "trajectory_xy.png"), dpi=220)
plt.close()

# ---------- 3) XZ trajectory ----------
plt.figure(figsize=(7, 5))
if np.any(valid_gt):
    plt.plot(gt_pos[valid_gt, 0], gt_pos[valid_gt, 2], label="GT")
if np.any(valid_est):
    plt.plot(est_pos[valid_est, 0], est_pos[valid_est, 2], label="Estimated")
plt.title("Camera trajectory in XZ")
plt.xlabel("x")
plt.ylabel("z")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(RESULTS_DIR, "trajectory_xz.png"), dpi=220)
plt.close()

# ---------- 4) YZ trajectory ----------
plt.figure(figsize=(7, 5))
if np.any(valid_gt):
    plt.plot(gt_pos[valid_gt, 1], gt_pos[valid_gt, 2], label="GT")
if np.any(valid_est):
    plt.plot(est_pos[valid_est, 1], est_pos[valid_est, 2], label="Estimated")
plt.title("Camera trajectory in YZ")
plt.xlabel("y")
plt.ylabel("z")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(os.path.join(RESULTS_DIR, "trajectory_yz.png"), dpi=220)
plt.close()

# ---------- 5) Position components vs frame ----------
frames = np.arange(len(gt_pos))

fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

labels = ["x", "y", "z"]
for i in range(3):
    axs[i].plot(frames[valid_gt], gt_pos[valid_gt, i], label="GT")
    axs[i].plot(frames[valid_est], est_pos[valid_est, i], label="Estimated")
    axs[i].set_ylabel(labels[i])
    axs[i].grid(True)

axs[0].set_title("Position components vs frame")
axs[-1].set_xlabel("frame")
axs[0].legend()
plt.tight_layout()
plt.savefig(os.path.join(RESULTS_DIR, "trajectory_components_vs_frame.png"), dpi=220)
plt.close(fig)

# ---------- 6) Position error vs frame ----------
pos_error = np.full(len(gt_pos), np.nan)
if np.any(valid_both):
    pos_error[valid_both] = np.linalg.norm(est_pos[valid_both] - gt_pos[valid_both], axis=1)

plt.figure(figsize=(8, 4))
valid_err = np.isfinite(pos_error)
plt.plot(frames[valid_err], pos_error[valid_err])
plt.title("Position error vs frame")
plt.xlabel("frame")
plt.ylabel("||p_est - p_gt||")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(RESULTS_DIR, "position_error_vs_frame.png"), dpi=220)
plt.close()

# ---------- 7) Relative rotation error ----------
rot_trace_err = arr(traj_eval.rot_trace_err).reshape(-1)
valid_pair = arr(traj_eval.valid_pair).reshape(-1).astype(bool)

plt.figure(figsize=(8, 4))
plt.plot(np.where(valid_pair)[0] + 1, rot_trace_err[valid_pair])
plt.title("Relative rotation error")
plt.xlabel("relative motion index")
plt.ylabel("trace(I - R_err)")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(RESULTS_DIR, "rot_error.png"), dpi=220)
plt.close()

# ---------- 8) Scale ratio ----------
scale_ratio = arr(traj_eval.scale_ratio).reshape(-1)
valid_scale = arr(traj_eval.valid_scale).reshape(-1).astype(bool)

plt.figure(figsize=(8, 4))
plt.plot(np.where(valid_scale)[0] + 1, scale_ratio[valid_scale])
plt.title("Scale ratio per relative motion")
plt.xlabel("relative motion index")
plt.ylabel("||t_est|| / ||t_gt||")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(RESULTS_DIR, "scale_ratio.png"), dpi=220)
plt.close()

# ---------- 9) 3D map scatter ----------
est_points = arr(map_eval.est_points)
gt_points = arr(map_eval.gt_points)

if est_points.size > 0 and gt_points.size > 0:
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(gt_points[:, 0], gt_points[:, 1], gt_points[:, 2], s=8, label="GT map")
    ax.scatter(est_points[:, 0], est_points[:, 1], est_points[:, 2], s=8, label="Estimated map")
    ax.set_title("3D map points in frame c0")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS_DIR, "map_scatter_3d.png"), dpi=220)
    plt.close(fig)

# ---------- 10) Map XY scatter ----------
if est_points.size > 0 and gt_points.size > 0:
    plt.figure(figsize=(7, 5))
    plt.scatter(gt_points[:, 0], gt_points[:, 1], s=12, label="GT map")
    plt.scatter(est_points[:, 0], est_points[:, 1], s=12, label="Estimated map")
    plt.title("Map points in XY")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS_DIR, "map_xy.png"), dpi=220)
    plt.close()

# ---------- 11) Map XZ scatter ----------
if est_points.size > 0 and gt_points.size > 0:
    plt.figure(figsize=(7, 5))
    plt.scatter(gt_points[:, 0], gt_points[:, 2], s=12, label="GT map")
    plt.scatter(est_points[:, 0], est_points[:, 2], s=12, label="Estimated map")
    plt.title("Map points in XZ")
    plt.xlabel("x")
    plt.ylabel("z")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(RESULTS_DIR, "map_xz.png"), dpi=220)
    plt.close()

print("Saved final plots in results/:")
for name in [
    "trajectory_3d.png",
    "trajectory_xy.png",
    "trajectory_xz.png",
    "trajectory_yz.png",
    "trajectory_components_vs_frame.png",
    "position_error_vs_frame.png",
    "rot_error.png",
    "scale_ratio.png",
    "map_scatter_3d.png",
    "map_xy.png",
    "map_xz.png",
]:
    path = os.path.join(RESULTS_DIR, name)
    if os.path.exists(path):
        print(" -", path)
