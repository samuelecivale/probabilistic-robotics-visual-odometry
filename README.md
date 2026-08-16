# Monocular Visual Odometry

A geometric **monocular visual odometry** pipeline for camera pose estimation and sparse 3D reconstruction.

The project implements the main components of a classical feature-based visual odometry system, including:

* relative-pose initialization;
* essential-matrix geometry;
* triangulation;
* landmark association;
* local-map management;
* camera pose refinement through reprojection error.

## Key Results

| Metric               |        Result |
| -------------------- | ------------: |
| Valid relative poses | **120 / 120** |
| Position RMSE        |   **4.39 cm** |
| Map RMSE             |  **11.27 cm** |

The pipeline successfully estimated a valid relative camera pose for **all 120 evaluated frame transitions**.

---

## Overview

Visual odometry estimates the motion of a camera from a sequence of images.

The objective of this project is to reconstruct both:

1. the **camera trajectory**;
2. a sparse representation of the observed **3D environment**.

The implemented pipeline follows a classical geometry-based approach rather than an end-to-end learned model.

```text
Image Sequence
      │
      ▼
Feature Correspondences
      │
      ▼
Geometric Initialization
      │
      ▼
Essential Matrix
      │
      ▼
Relative Camera Pose
      │
      ▼
Triangulation
      │
      ▼
Local 3D Map
      │
      ▼
Data Association
      │
      ▼
Pose Refinement
      │
      ▼
Estimated Trajectory + Map
```

---

## Pipeline

### 1. Geometric Initialization

The first stage estimates the relative motion between camera frames from matched image features.

The **essential matrix** encodes the epipolar constraint between corresponding normalized image points:

$$
\mathbf{x}'^{T}\mathbf{E}\mathbf{x} = 0
$$

where:

* $\mathbf{x}$ is a feature observation in the first frame;
* $\mathbf{x}'$ is the corresponding observation in the second frame;
* $\mathbf{E}$ is the essential matrix.

The matrix is decomposed to obtain candidate relative rotations and translations.

The physically valid configuration is selected by verifying the reconstructed scene geometry.

---

## 2. Relative Pose Estimation

The relative transformation between consecutive camera poses can be represented as:

$$
\mathbf{T}
==========

\begin{bmatrix}
\mathbf{R} & \mathbf{t} \
\mathbf{0}^{T} & 1
\end{bmatrix}
$$

where:

* $\mathbf{R}\in SO(3)$ is the relative camera rotation;
* $\mathbf{t}\in\mathbb{R}^{3}$ is the relative translation.

Successive relative transformations are composed to reconstruct the camera trajectory.

---

## 3. Triangulation

Once relative camera motion is available, corresponding image observations are used to reconstruct 3D landmarks.

Conceptually:

```text
Camera 1                  Camera 2
   \                         /
    \                       /
     \                     /
      \                   /
       \                 /
        \               /
          3D Landmark
```

Each landmark is estimated from multiple image observations and becomes part of the map used for subsequent localization.

---

## 4. Local Active Map

Instead of relying indiscriminately on every reconstructed landmark, the system maintains a **local active map** containing landmarks currently useful for camera tracking.

This helps keep the localization problem focused on geometrically relevant points.

The active map is updated as the camera moves through the environment.

```text
Global reconstructed landmarks
             │
             ▼
     Visibility / relevance
             │
             ▼
       Active local map
             │
             ▼
       Camera tracking
```

---

## 5. Data Association

For every new frame, image observations must be associated with existing 3D landmarks.

The project uses geometric consistency and projection information to establish these correspondences.

This stage is critical because incorrect 2D–3D associations directly affect pose estimation.

The pipeline includes a **projective ICP-style association procedure**, using the current pose estimate to project map points into the image and identify compatible observations.

---

## 6. Pose Refinement

Once 2D image observations have been associated with 3D landmarks, the camera pose is refined by minimizing reprojection error.

For a 3D landmark $\mathbf{P}_i$ and corresponding image observation $\mathbf{u}_i$, the reprojection error can be expressed as:

$$
\mathbf{e}_i
============

## \mathbf{u}_i

\pi
\left(
\mathbf{R}\mathbf{P}_i + \mathbf{t}
\right)
$$

where $\pi(\cdot)$ denotes the camera projection function.

The refined pose is obtained by minimizing:

$$
\min_{\mathbf{R},\mathbf{t}}
\sum_i
\left|
\mathbf{e}_i
\right|^2
$$

This optimization aligns the projected landmarks with their measured image positions.

---

## Complete Architecture

```text
                 ┌────────────────────┐
                 │   Monocular Images │
                 └─────────┬──────────┘
                           │
                           ▼
                 ┌────────────────────┐
                 │ Feature Matching   │
                 └─────────┬──────────┘
                           │
                           ▼
                 ┌────────────────────┐
                 │ Essential Matrix   │
                 │ / Relative Pose    │
                 └─────────┬──────────┘
                           │
                           ▼
                 ┌────────────────────┐
                 │   Triangulation    │
                 └─────────┬──────────┘
                           │
                           ▼
                 ┌────────────────────┐
                 │    Local Map       │
                 └─────────┬──────────┘
                           │
                           ▼
                 ┌────────────────────┐
                 │ Data Association   │
                 │ / Projective ICP   │
                 └─────────┬──────────┘
                           │
                           ▼
                 ┌────────────────────┐
                 │ Reprojection-Based │
                 │ Pose Refinement    │
                 └─────────┬──────────┘
                           │
                  ┌────────┴────────┐
                  ▼                 ▼
          Camera Trajectory      3D Map
```

---

# Results

## Relative Pose Estimation

The pipeline produced:

> **120 valid relative poses out of 120 evaluated pose transitions.**

This indicates that the geometric initialization and tracking pipeline remained operational throughout the evaluated sequence.

---

## Trajectory Accuracy

The estimated camera trajectory achieved a:

> **Position RMSE of 4.39 cm**

The position RMSE summarizes the translational discrepancy between the estimated and reference camera trajectories.

---

## Mapping Accuracy

The reconstructed sparse map achieved a:

> **Map RMSE of 11.27 cm**

This measures the geometric error of the reconstructed 3D landmarks relative to the reference map.

---

## Results Summary

```text
Relative poses : 120 / 120 valid
Position RMSE  : 0.0439 m
Map RMSE       : 0.1127 m
```

These results show that the pipeline is able to combine geometric initialization, tracking, mapping, and nonlinear refinement into a complete visual-odometry system.

---

# Why This Project Matters

The project implements several of the geometric building blocks underlying modern robotic perception systems.

Unlike an end-to-end pose-regression approach, the estimation process remains interpretable:

```text
Measurements
     ↓
Geometry
     ↓
Correspondences
     ↓
Optimization
     ↓
State Estimate
```

This makes it possible to reason explicitly about:

* camera geometry;
* pose estimation;
* landmark uncertainty;
* data association;
* reprojection error;
* mapping;
* accumulated odometry drift.

---

# Visual Odometry vs SLAM

The focus of this project is **visual odometry**.

The system estimates local camera motion and reconstructs a map that supports tracking.

A complete SLAM system would typically add mechanisms such as:

* loop-closure detection;
* global map optimization;
* pose-graph optimization;
* long-term map consistency.

These components are outside the main scope of this implementation.

---

# Tech Stack

`MATLAB / Octave` · `Computer Vision` · `Visual Odometry` · `3D Geometry` · `Pose Estimation` · `Optimization` · `Robotics`

---

# Concepts Implemented

This project covers:

* monocular visual odometry;
* projective geometry;
* epipolar geometry;
* essential matrix estimation;
* relative camera pose;
* homogeneous transformations;
* triangulation;
* sparse 3D reconstruction;
* data association;
* local mapping;
* projective ICP;
* reprojection error minimization;
* trajectory estimation;
* quantitative localization evaluation.

---

# Limitations

As a monocular visual-odometry system, the implementation inherits several limitations typical of purely vision-based geometric estimation.

In particular:

* estimation quality depends on reliable visual correspondences;
* poor image texture can degrade tracking;
* incorrect data association can affect camera localization;
* errors accumulate over time without global correction;
* no global loop-closure mechanism is included;
* monocular geometry does not directly provide absolute metric scale without additional information or assumptions.

The project should therefore be interpreted as an implementation and evaluation of a geometric visual-odometry pipeline rather than a production-ready SLAM system.

---

# Possible Extensions

Future extensions could include:

* robust estimation with additional outlier rejection;
* bundle adjustment;
* keyframe selection;
* global map optimization;
* loop closure;
* pose-graph optimization;
* ORB or other modern local descriptors;
* stereo visual odometry;
* visual-inertial odometry;
* ROS integration;
* real-time implementation in C++.

---

# Project Context

This project was developed within the **Probabilistic Robotics** coursework of the MSc in Artificial Intelligence and Robotics at Sapienza University of Rome.

The objective was to implement and understand the geometric estimation pipeline underlying camera-based robot localization and mapping.

---

# Author

**Samuele Civale**
MSc Artificial Intelligence and Robotics
Sapienza University of Rome

GitHub: [@samuelecivale](https://github.com/samuelecivale)
