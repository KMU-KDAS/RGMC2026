<p align="center">
  <img src="../../images/t1_geometry01.png" alt="Task 1 geometry and shape alignment overview" width="950"/>
</p>

# Geometry and Shape Alignment for Planar Pushing

**Task 1 · Model-Based Closed-Loop Planar Pushing · Team K-DAS**

This module converts the object center, orientation, contour, and keypoints obtained from observation into a **workspace geometry state** that can be used by the planner, and aligns the current and target shapes under a common geometric convention.

In Task 1, matching only the object center is not sufficient. For square and T-shaped objects, orientation error directly affects both the actual contact location and IoU. For asymmetric shapes, even the definition of the object center can shift the reconstructed target position. K-DAS therefore represents objects using polygons and poses, with separate alignment procedures that account for shape-specific symmetry and differences in reference axes.

> This document focuses on **shape representation, current-target alignment, and IoU computation in workspace coordinates**, rather than image-space detection. Pixel-to-workspace coordinate conversion is described separately in the shared `mapping/README.md`.

---

## 1. Role in the Task 1 System

<p align="center">
  <img src="../../images/t1_geometry02.png" alt="Geometry and shape alignment pipeline" width="1000"/>
</p>

```text
Perception contour / keypoints
        ↓
Mapping to workspace coordinates
        ↓
Build pose and polygon representation
        ↓
Apply shape-specific normalization
        ↓
Align current and target geometry
        ↓
Compute pose error, shape error and IoU
        ↓
Pass normalized state to planning and evaluation
```

The main output of this module can be represented as follows.

```python
geometry_state = {
    "shape_type": "circle | square | t_shape | unknown",
    "center_ws": [x, y],
    "orientation_rad": theta,
    "polygon_ws": [[x0, y0], ...],
    "keypoints_ws": [[x0, y0], ...],
    "symmetry_order": 1 | 4,
    "alignment_metadata": {...},
}
```

Related documents:

- Task 1 overview: [`../README.md`](../README.md)
- Planning and candidate selection: [`../planning/README.md`](../planning/README.md)
- Shared mapping: [`../../mapping/README.md`](../../mapping/README.md)

---

## 2. Pose and Polygon Representation

The object pose is represented by its center position and rotation angle on the workspace plane.

$$
q=[x,\ y,\ \theta]
$$

Each vertex $p_i^{local}$ of a local template is transformed into workspace coordinates by rotation and translation.

$$
R(\theta)=
\begin{bmatrix}
\cos\theta & -\sin\theta\\
\sin\theta & \cos\theta
\end{bmatrix}
$$

$$
p_i^{world}=c+R(\theta)p_i^{local}
$$

Here, $c=[x,y]^T$ is the object center. This representation allows the planner to handle translation and orientation separately while also computing the actual region occupied by the polygon in the workspace.

### 2.1 Shape-specific geometry

| Shape | Geometry representation | Orientation handling |
|---|---|---|
| Circle | center, radius, sampled boundary points | orientation has low weight because rotation has little geometric effect |
| Square | ordered four vertices | 90° rotational symmetry is handled explicitly |
| T-shape | ordered eight vertices | reference axis and vertex correspondence are important because the shape is asymmetric |
| Unknown / surprise shape | detected contour or simplified polygon | reduce template hard-coding where possible and use contour-level comparison |

By representing square and T-shaped objects as vertex sets, and circles as uniformly sampled boundary points, different object types can be compared using a common set of polygon operations.

---

## 3. IoU as the Shape-level Success Metric

<p align="center">
  <img src="../../images/t1_geometry03.png" alt="IoU-based shape comparison" width="700"/>
</p>

The overlap between the current shape $S_{current}$ and target shape $S_{target}$ is measured using IoU.

$$
\mathrm{IoU}=\frac{\mathrm{Area}(S_{current}\cap S_{target})}
{\mathrm{Area}(S_{current}\cup S_{target})}
$$

- `IoU = 1.0`: the two shapes overlap completely
- `IoU = 0.0`: the two shapes do not overlap at all

IoU was used for the following reasons.

1. It captures both center-position error and orientation error in a single shape-level metric.
2. It distinguishes squares and T-shapes with the same center but different orientations.
3. It connects the planner's internal pose error with the actual shape overlap used by the competition evaluator.
4. It can also be applied to surprise shapes as long as a contour is available.

However, using IoU alone for candidate selection can be sensitive to small shape changes and provides little useful gradient when the current and target objects are far apart. The planner therefore combines pose error with reference-path progress internally, while IoU is used mainly for success evaluation and final shape-quality checks.

---

## 4. Why Shape Alignment Required a Separate Module

Even when perception is stable, an incorrect target pose can be produced if the detector and planner interpret the same shape using different geometric conventions.

The main issues were:

- different reference axes between detector orientation and canonical template orientation
- different center definitions, such as vertex-average center versus polygon area centroid
- loss of geometric information when a T-shaped target is reduced to only one center and one angle
- identical square poses being returned with different starting vertex indices

These errors cannot be corrected by the physics model or path planner. Current and target geometry must first be normalized under the same convention before generating candidate actions.

---

## 5. T-shape Basis Mismatch

<p align="center">
  <img src="../../images/t1_geometry04.png" alt="T-shape detector and planner basis mismatch" width="900"/>
</p>

In the initial T-shape planner, the target orientation shown in visualization did not match the orientation that the planner was internally trying to align with. The cause was a difference in how orientation was defined between the two modules.

- **Detector**: computes orientation from the stem direction estimated from the image contour
- **Planner**: interprets orientation using the axis of the canonical `T_BASE` shape defined in the configuration

The two conventions differed by approximately 90°. This issue was less visible for circles, which have no meaningful orientation, and squares, which have 90° rotational symmetry. For the asymmetric T-shape, however, it produced a clear target mismatch.

### 5.1 Correction rule

Initially, a 90° correction was applied only to the current observation. Because the same correction was not applied to the target, the two shapes were still being compared in different frames.

The final implementation applied the same basis correction to both the current and target shapes.

$$
\theta_{planner}=\theta_{detector}+\frac{\pi}{2}
$$

The important point is that the 90° correction is not a general rule for T-shaped objects. It compensates for the **specific reference-axis difference between the detector and the canonical template**. In a public implementation, this offset should be stored as part of the shape configuration.

---

## 6. Why Centroid-only Target Reconstruction Failed

<p align="center">
  <img src="../../images/t1_geometry05.png" alt="Raw T-shape target and reconstructed planner target" width="780"/>
</p>

The initial implementation did not pass all eight T-shape target vertices provided by the competition directly to the planner. Instead, the target was first reduced to a center point and rotation angle, and the internal canonical T template was then placed at that reconstructed pose.

During this process, the center definition used to estimate the target pose differed from the reference point used to place the canonical template.

<p align="center">
  <img src="../../images/t1_geometry06.png" alt="Difference between vertex-average center and polygon area centroid" width="620"/>
</p>

For the internal T template, the following difference was observed.

- Vertex-average center: approximately **2.65 mm**
- Polygon area centroid: approximately **4.24 mm**
- Difference between the two center definitions: approximately **1.59 mm**

A 1.59 mm difference may appear small, but for the compact CloudGripper objects it shifts the entire reconstructed target polygon. Because the T-shape is asymmetric, the center definition directly affects target alignment.

---

## 7. Eight-point Rigid Alignment for T-shape

Instead of reconstructing the target from a single center and angle, the planner was modified to use all eight target vertices directly.

<p align="center">
  <img src="../../images/t1_geometry07.png" alt="Eight-point rigid alignment objective" width="650"/>
</p>

Let $p_i$ denote the vertices of the canonical T template and $q_i$ the vertices of the actual target. The alignment solves for the rotation and translation that best overlap the two shapes.

$$
R^\star,t^\star=\arg\min_{R,t}\sum_i \lVert Rp_i+t-q_i\rVert^2
$$

This alignment has the following properties.

* It uses the **entire shape**, rather than a single center point.
* It preserves more of the target's actual scale and asymmetric structure.
* It estimates rotation and translation together.
* It reduces the offset between the planner's canonical shape and the evaluator target.

<p align="center">
  <img src="../../images/t1_geometry08.png" alt="T-shape target alignment before and after correction" width="900"/>
</p>

### 7.1 Required preprocessing

For rigid alignment to remain stable, the eight current and target vertices must follow a consistent ordering. During perception, keypoints including the concave corners are ordered first, and the geometry module then builds correspondences using the same canonical vertex order.

### 7.2 Development-stage validation

<p align="center">
  <img src="../../images/t1_geometry09.png" alt="T-shape alignment test examples" width="900"/>
</p>

In the initial Week 7 validation, some runs reached high IoU, but overall performance was still unstable.

| Metric                  | Development-stage result |
| ----------------------- | -----------------------: |
| Mean max IoU            |               **0.5896** |
| Runs reaching IoU ≥ 0.8 |               **2 / 10** |
| Best observed max IoU   |               **0.8698** |
| Mean step at max IoU    |                 **10.1** |

These results showed that geometry correction alone was not enough to complete the full Task 1 system. After further improvements to candidate scoring, approach paths, and execution time, T and T-long achieved final Top-3 average scores of **52.24** and **57.21**, respectively.

---


---

---

## 8. Square Symmetry Problem

<p align="center">
  <img src="../../images/t1_geometry10.png" alt="Square alignment examples with vertex-order ambiguity" width="850"/>
</p>

A square has a meaningful orientation but also has 90° rotational symmetry. Even for the same physical pose, the first vertex returned by the contour detector can change from frame to frame.

For example, if the current vertex order is `[1,2,3,4]` and the target is returned as `[2,3,4,1]`, the two polygons may already represent the same physical alignment. If the indices are compared directly without accounting for this shift, a large shape error can be computed and the planner may unnecessarily rotate a square that is already aligned.

---

## 9. Cyclic Vertex Matching for Square

For squares, the target vertex sequence is compared under all four cyclic shifts, and the correspondence with the smallest mean distance error is selected.

$$
E_{square}=\min_{k\in\{0,1,2,3\}}
\frac{1}{4}\sum_{i=0}^{3}
\left\|p_i-q_{(i+k)\bmod4}\right\|_2
$$

```python
best_error = float("inf")

for shift in range(4):
    rolled_target = np.roll(target_points, shift=shift, axis=0)
    error = mean_distance(current_points, rolled_target)
    best_error = min(best_error, error)
```

<p align="center">
  <img src="../../images/t1_geometry11.png" alt="Square cyclic vertex matching" width="900"/>
</p>

This method treats different starting vertex indices as the same physical alignment. It was particularly important near the goal, where it reduced unnecessary corrective rotations.

> The current implementation compares cyclic shifts while preserving the same winding direction. In environments where the detector may switch between clockwise and counter-clockwise ordering, winding normalization should be applied first.

---

## 10. Square Validation Results

<p align="center">
  <img src="../../images/t1_geometry12.png" alt="Square validation table" width="850"/>
</p>

Across 11 tests after applying the square symmetry correction and execution-path improvements, every run achieved a final IoU of at least 0.8.

| Metric | Result |
|---|---:|
| Successful runs, final IoU ≥ 0.8 | **11 / 11** |
| Best final IoU | **0.90** |
| Average final score | **103.22** |
| Average number of steps | **4.0** |
| Average total time | **97.65 s** |
| Average time per step | **24.41 s** |

Runs 10 and 11 both reached a final IoU of 0.90, and Run 11 satisfied the success threshold in 2 steps and 46.01 seconds. These results indicate that, when integrated with the full planner, square geometry normalization reduced repeated unnecessary rotations and made near-goal alignment more stable.

However, these results are not an isolated ablation of the symmetry correction. Approach-path waypoint limits and execution-time improvements were introduced during the same development period, so the results should be interpreted as validation of the complete system configuration.

---

## 11. Handling Surprise Shapes

The final evaluation included Plus and Organic objects as surprise shapes. These objects have more complex concavity than the T-shape or do not have an obvious canonical axis.

K-DAS handled these objects using the following principles.

- preserve the detected contour directly as a polygon whenever possible
- avoid reducing the shape too aggressively to a single center and orientation
- sample candidate contact points directly from the contour
- use IoU and contour-level geometry as common evaluation criteria

The final Top-3 average scores were **34.47** for Plus and **37.82** for Organic. Although these scores were lower than those of the seen objects, maintaining a common contour-based interface rather than assuming only fixed templates allowed the system to execute valid pushes on unseen objects.

<p align="center">
  <img src="../../images/t1_geometry14.png" alt="Task 1 object-wise final performance" width="850"/>
</p>

---

## 12. Interface to Planning

The geometry module provides the planner with the following information.

```python
aligned = {
    "current_pose": [x, y, theta],
    "target_pose": [x_goal, y_goal, theta_goal],
    "current_polygon": current_polygon_ws,
    "target_polygon": target_polygon_ws,
    "pose_error": [dx, dy, dtheta],
    "shape_error": point_set_error,
    "iou": current_iou,
    "symmetry_shift": selected_shift,
}
```

The planner uses this state to:

- generate a reference pose path
- compute the predicted next polygon for each candidate
- evaluate pose and shape error relative to the target
- determine whether the success threshold has been reached
- decide whether near-goal correction is required

By separating geometry from planning, the current-target alignment rules remain consistent even when the physics model or candidate generator is modified.

---

## 13. Failure Cases and Safeguards

### 13.1 Wrong vertex ordering

If the eight-point correspondence for the T-shape becomes inconsistent, rigid alignment can select a completely different rotation. Vertex-order validation and polygon winding normalization are required.

### 13.2 Basis correction applied to one side only

If the correction is applied only to the current shape but not the target, the system compares two different coordinate conventions. The same basis correction must be applied to both current and target geometry.

### 13.3 Template over-simplification

Reducing an asymmetric or surprise shape to only one center and orientation loses information from the actual contour. The target polygon itself should be preserved whenever possible.

### 13.4 Symmetry ignored

For symmetric objects such as squares, fixed vertex-index comparison can produce unnecessary rotation actions. Shape-specific symmetry groups should be managed through configuration.

### 13.5 IoU-only optimization

When the initial overlap is close to zero, IoU alone provides little information for distinguishing action quality. Pose error, progress, and path feasibility should be considered together.

---

## 14. Suggested Repository Layout

```text
task1/
└─ geometry/
   ├─ README.md
   ├─ pose.py
   ├─ polygon.py
   ├─ iou.py
   ├─ alignment.py
   ├─ symmetry.py
   ├─ shape_configs/
   │  ├─ circle.yaml
   │  ├─ square.yaml
   │  └─ t_shape.yaml
   ├─ tests/
   │  ├─ test_iou.py
   │  ├─ test_t_alignment.py
   │  └─ test_square_symmetry.py
```

The following items are appropriate to define in each shape configuration.

- canonical keypoints
- orientation basis offset
- expected vertex count
- symmetry order
- winding convention
- IoU / success threshold
- target-pose reconstruction rule

---


> All visual materials are managed in the repository-level `images/` directory.

## 15. Takeaway

The Task 1 geometry module is not simply a preprocessing step that converts contours into polygons. It is the **alignment layer that ensures the detector, evaluator, and planner interpret object orientation and center using the same geometric convention**.

For the T-shape, K-DAS identified a 90° detector-planner basis mismatch and a difference in center definitions, then preserved the target geometry using rigid alignment over all eight vertices. For squares, cyclic vertex matching was introduced to account for 90° symmetry and reduce unnecessary rotations of objects that were already aligned.

> **Accurate pushing does not begin with a good physics model alone. It begins by representing the current and target shapes under the same geometric convention.**
