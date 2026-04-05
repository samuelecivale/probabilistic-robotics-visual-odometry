# Visual Odometry: Probabilistic Robotics Project

Visual odometry pipeline in Octave. Given a sequence of image measurements with feature descriptors and camera calibration, the system estimates the camera trajectory and builds a sparse 3D map.

## What this does

The pipeline takes the provided dataset (camera intrinsics, image measurements with appearance descriptors) and:

1. Initializes from the first two frames, estimates the relative pose via Essential matrix decomposition and triangulates an initial set of 3D points
2. Tracks the camera incrementally through the sequence using projective ICP (reprojecting known 3D points into the current frame and matching by appearance)
3. Triangulates new landmarks along the way to keep the active set fresh
4. Refines each pose by minimizing reprojection error

Data association is done by exhaustive comparison of the 10, dimensional appearance descriptors, not efficient, but reliable enough for this dataset.

## What actually made it work

The main thing that improved results was switching from matching against the full accumulated map to using a **local active cloud**: basically a sliding window of recently triangulated points. Matching against the whole map was drifting too much; keeping it local made tracking way more stable.

Each pose is also refined with a least-squares step on the reprojection error, and there's a robust initialization to filter out bad correspondences before refinement.

## Dataset

The provided dataset contains:

- `camera.dat`: camera intrinsics (camera matrix, near/far planes, image size)
- `meas-XXXX.dat`: per-frame measurements: point IDs, image coordinates [col, row], and 10D appearance vectors
- `trajectory.dat`: ground truth poses (used only for evaluation)
- `world.dat`: ground truth 3D landmarks (used only for evaluation)

## How to run

```bash
# run the VO pipeline
octave-cli --silent main_vo_active.m

# evaluate against ground truth
octave-cli --silent main_evaluate_vo.m
```

First script produces `results/vo_active_results.mat`, second one produces `results/evaluation_results.mat` and all the plots.

## Evaluation

For each consecutive pair of estimated poses \(T_i, T_{i+1}\), I compute the relative motion and compare it with the corresponding relative ground-truth motion.

The estimated relative motion is:
\[
T_{\mathrm{rel}} = T_i^{-1} T_{i+1}
\]

The ground-truth relative motion is:
\[
T_{\mathrm{rel}}^{\mathrm{GT}} = (T_i^{\mathrm{GT}})^{-1} T_{i+1}^{\mathrm{GT}}
\]

The relative error transformation is:
\[
T_{\mathrm{err}} = T_{\mathrm{rel}}^{-1} T_{\mathrm{rel}}^{\mathrm{GT}}
\]

### Rotation error

The rotation error is computed as:
\[
e_R = \operatorname{trace}\!\left(I - R_{\mathrm{err}}\right)
\]
where \(R_{\mathrm{err}}\) is the rotational part of \(T_{\mathrm{err}}\).

### Translation scale consistency

Since the estimated trajectory is only defined up to scale, I evaluate the translation through the ratio:
\[
r_t = \frac{\|t_{\mathrm{rel}}\|}{\|t_{\mathrm{rel}}^{\mathrm{GT}}\|}
\]

I then check how consistent this ratio remains over the whole sequence.

### Map evaluation

The estimated map is scaled using the inverse of the median translation ratio:
\[
s = \frac{1}{\operatorname{median}(r_t)}
\]

The corrected map is then:
\[
p_i^{\mathrm{corr}} = s \, p_i
\]

Finally, I compute the RMSE between the corrected estimated landmarks and the corresponding ground-truth landmarks:
\[
\mathrm{RMSE}_{\mathrm{map}} =
\sqrt{\frac{1}{N}\sum_{i=1}^{N}
\left\|p_i^{\mathrm{corr}} - p_i^{\mathrm{GT}}\right\|^2}
\]

## Numerical results

```
Valid relative pose pairs:    120 / 120
Mean rotation trace error:    0.000000
Median rotation trace error:  0.000000
Mean scale ratio:             5.020486
Median scale ratio:           4.989554
Scale ratio std:              0.042870
RMSE position:                0.0439 m
Matched map landmarks:        410
Map RMSE:                     0.1127 m
```

Rotation estimation is basically perfect, scale is very consistent across the sequence (std ~ 0.04), and position RMSE after scale correction is under 5 cm. The map aligns reasonably well with GT.

## Plots

### Trajectory
| | | |
|---|---|---|
| ![](results/trajectory_3d.png) | ![](results/trajectory_xy.png) | ![](results/position_error_vs_frame.png) |
| 3D trajectory | XY plane | Position error per frame |

### Rotation and scale
| | |
|---|---|
| ![](results/rot_error.png) | ![](results/scale_ratio.png) |
| Rotation trace error | Scale ratio over sequence |

### Map
| | | |
|---|---|---|
| ![](results/map_scatter_3d.png) | ![](results/map_xy.png) | ![](results/map_xz.png) |
| 3D reconstruction | XY view | XZ view |

## Repo structure

```
.
├── data/                    # dataset
├── results/                 # output .mat files and plots
├── src/                     # source code
├── main_vo_active.m         # main pipeline
├── main_evaluate_vo.m       # evaluation script
└── README.md
```
