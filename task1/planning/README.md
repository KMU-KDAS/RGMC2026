<p align="center">
  <img src="../../images/t1_planning01.png" alt="Task 1 planning and candidate selection pipeline" width="980"/>
</p>

# Planning and Candidate Selection for Planar Pushing

**Task 1 · Model-Based Closed-Loop Planar Pushing · Team K-DAS**

This module determines **where to push, in which direction, and by how much** based on the current object state and the target shape. It generates multiple push candidates, predicts each candidate with a 1-step rigid-body dynamics model, and evaluates the result against a candidate-specific intermediate target on the reference path. Candidates that the real robot cannot safely approach are then removed before selecting the final action.

> The goal is not to find the single largest push, but to **repeatedly select small, executable improvements**. After every push, the object is re-observed and the entire planning process is performed again.

---

## 1. Role in the Task 1 System

```text
Perception / Geometry state
        ↓
Reference path from current pose to target pose
        ↓
Contact × direction × stroke candidate generation
        ↓
One-step dynamics prediction
        ↓
Candidate-specific progress and risk scoring
        ↓
Direct / L / U / A* safe approach path attachment
        ↓
Approach → push → retreat → re-observe → replan
```

Related documents:

- Task 1 overview: [`../README.md`](../README.md)
- Geometry and alignment: [`../geometry/README.md`](../geometry/README.md)
- Rigid-body dynamics: [`../dynamics/README.md`](../dynamics/README.md)
- Shared mapping: [`../../mapping/README.md`](../../mapping/README.md)

---

## 2. Why a Reference Path Was Needed

If candidates are evaluated only against the final target pose, every candidate is compared with the same distant goal regardless of how far a single push can realistically move the object. This can undervalue short but stable candidates, while long candidates may appear favorable even when they carry a high risk of overshoot.

To address this, a smooth reference path was generated between the current and target poses, and an **intermediate target** was selected according to the expected motion of each candidate.

### 2.1 Minimum-jerk interpolation

The object pose is represented as:

$$
q=[x,\ y,\ \theta]
$$

For normalized time $t\in[0,1]$, the interpolation function is:

$$
s(t)=10t^3-15t^4+6t^5
$$

$$
q_{ref}(t)=q_{current}+s(t)(q_{target}-q_{current})
$$

This fifth-order interpolation provides a reference trajectory whose velocity and acceleration do not change abruptly at the start and end points. The orientation difference is wrapped to the range $[-\pi,\pi)$ so that the shortest rotational direction is used.

<p align="center"><img src="../../images/t1_planning02.png" alt="Reference path examples for circle, square, and T-shape" width="820"/></p>

### 2.2 Candidate-specific intermediate target

Let the predicted 1-step pose of candidate $a_j$ be $\hat q_{t+1}^{(j)}$. The reference point that best corresponds to that candidate is selected as:

$$
k_j^*=\arg\min_k d\left(\hat q_{t+1}^{(j)},q_{ref,k}\right)
$$

Instead of comparing every candidate with one fixed intermediate target, each candidate is evaluated against the **reference-path point it can realistically reach**. This reduces bias when candidates have different motion magnitudes and makes the evaluation more stable.

---

## 3. Push Candidate Generation

An action is defined by three elements:

$$
a=(p_{contact},\ d_{push},\ L_{stroke})
$$

- `contact`: where on the object to push
- `direction`: in which direction to push
- `stroke`: how far to push

### 3.1 Contact points

For circular objects, the boundary was divided into 16 directions. For polygonal objects, contact candidates were placed at the 10%, 30%, 50%, 70%, and 90% positions along each edge.

<p align="center"><img src="../../images/t1_planning03.png" alt="Contact candidates for circle, square, and T-shape" width="820"/></p>

### 3.2 Translation and rotation candidates

At each contact point, three basic push directions were generated.

$$
d_{normal}=-\hat n
$$

$$
d_{spin,left}=-\hat n+\lambda\hat t
$$

$$
d_{spin,right}=-\hat n-\lambda\hat t
$$

The normal-direction candidate primarily encourages translation, while the candidates with tangential components are intended to produce both translation and rotation.

<p align="center"><img src="../../images/t1_planning04.png" alt="Normal and spin push candidates" width="820"/></p>

### 3.3 Distance-aware stroke

Longer strokes were used when the object was far from the target, and shorter strokes were used as it approached the goal. Continuing to use long pushes during final correction increases the risk of overshoot and workspace exit.

The improved policy used approximately the following ranges.

| Remaining distance | Stroke policy |
|---:|---|
| More than 5 cm | Long stroke |
| 1–5 cm | Medium stroke |
| 1 cm or less | Short correction stroke |

---

## 4. One-step Prediction Interface

Every generated candidate is rolled out once through the rigid-body dynamics model before execution.

$$
\hat q_{t+1}^{(j)}=f_{dyn}(q_t,a_j;\phi)
$$

Here, $\phi$ includes parameters such as mass, COM, moment of inertia, contact friction, and ground friction. The planning module receives the predicted next pose and polygon from the dynamics model and uses them to evaluate candidate progress and risk.

This prediction is not intended as a precise long-horizon simulation. It is a **short-horizon predictor used to compare the relative ranking of candidates from the same state**.

---

## 5. Candidate-wise Scoring

<p align="center"><img src="../../images/t1_planning05.png" alt="Candidate action scoring examples" width="820"/></p>

Candidate evaluation does not minimize a single distance metric. Instead, it considers the following factors together.

1. **Progress**: whether the error along the reference path actually decreases
2. **Direction agreement**: whether the predicted motion agrees with the target direction
3. **Overshoot risk**: whether the candidate moves beyond an appropriate reference point
4. **Workspace margin**: whether the predicted shape becomes too close to the workspace boundary
5. **Approach cost**: whether the robot can reach the start point safely and with a short path

Conceptually, the candidate score can be written as:

$$
S_j=w_p\Delta E_j+w_dC_{dir,j}
-w_oP_{over,j}-w_bP_{boundary,j}-w_aC_{approach,j}
$$

Here, $\Delta E_j$ is the difference between the current error and the predicted error after the push. Direction agreement can be computed using cosine similarity between the target-direction vector $g$ and predicted-motion vector $m_j$.

$$
C_{dir,j}=\frac{g\cdot m_j}{\|g\|\|m_j\|}
$$

In practice, the importance of each term can vary with shape type and distance to the target. For a public implementation, the weights and thresholds should therefore be separated into configuration files.

---

## 6. Computational Funnel

<p align="center"><img src="../../images/t1_planning06.png" alt="Candidate evaluation funnel" width="900"/></p>

Running A* for every candidate from the beginning would add unnecessary planning cost. The system therefore performs inexpensive checks first and postpones expensive approach-path search until after shortlisting.

```text
candidate generation
→ basic geometry / boundary rejection
→ one-step dynamics rollout
→ reference-path score
→ shortlist
→ safe approach path attachment
→ final executable candidate
```

This structure progressively separates candidates that are **physically promising but not executable** from those that are executable but provide little progress toward the goal.

---

## 7. Safe Approach Path Planning

Even if a push is physically favorable, it cannot be executed if the gripper cannot reach the start point without colliding with the object. Approach paths are therefore checked from the simplest method to more complex alternatives.

<p align="center"><img src="../../images/t1_planning07.png" alt="Direct, L, U, and A-star path hierarchy" width="940"/></p>

### 7.1 Direct → L-shape → U-shape → Short A*

| Priority | Path type | Purpose |
|---:|---|---|
| 1 | Direct | shortest and fastest straight-line approach |
| 2 | L-shape | avoid the obstacle with one turn |
| 3 | U-shape | make a larger detour around the object bounding box |
| 4 | Short A* | grid-based search when all geometric paths fail |

The basic A* evaluation function is:

$$
f(n)=g(n)+h(n)
$$

- $g(n)$: actual movement cost from the start point to the current node
- $h(n)$: estimated cost from the current node to the goal

### 7.2 Progressive waypoint limits

Limiting every A* path to exactly 5 waypoints reduced execution time, but it also rejected feasible paths that required 6 or more waypoints. To avoid this, the waypoint limit was relaxed progressively.

```python
for max_waypoints in [5, 7, 9, 12]:
    feasible = attach_paths(candidates, max_waypoints=max_waypoints)
    if feasible:
        break
```

<p align="center"><img src="../../images/t1_planning08.png" alt="Progressive A-star waypoint relaxation and path simplification" width="780"/></p>

If a short path is available, the search stops immediately at the 5-waypoint limit. Longer detours are allowed only when all shorter alternatives fail. The generated path is then simplified by checking whether intermediate waypoints can be skipped without collision.

### 7.3 Start state near the safety margin

After a push, the gripper may remain close to the object. In the next planning cycle, the start point can then be classified as being inside the inflated obstacle, causing a `no plan` result.

- If the start point is **inside the actual object polygon**, it is considered unsafe and is not allowed.
- If it is outside the real object but only overlaps the safety margin, the robot can first move to a safe location outside the inflated region before normal path planning begins.

<p align="center"><img src="../../images/t1_planning09.png" alt="Task 1 planning geometry in workspace" width="620"/></p>

---

## 8. Execution Plan and Closed-loop Replanning

The selected plan is executed in the following order.

| Stage | Description |
|---|---|
| Approach | move near the push start point through the planned waypoints |
| Align | align with the exact start point and push direction |
| Push | move the object by the selected stroke length |
| Retreat | move away from the object before the next planning cycle |
| Re-observe | measure the actual object pose and robot state again |
| Replan | regenerate and evaluate candidates from the new state |

<p align="center"><img src="../../images/t1_planning10.png" alt="Planned push and observed result" width="940"/></p>

After each push, the predicted pose is compared with the actual observed pose. The measured translation and rotation response can then be used in the next planning cycle and COM belief update. As a result, a single inaccurate 1-step prediction does not determine all subsequent actions.

---

## 9. Execution-time Optimization

In the remote robot environment, overall performance was limited not only by computation time but also by repeated waiting time at each waypoint.

<p align="center"><img src="../../images/t1_planning11.png" alt="Approach path sleep-time optimization" width="850"/></p>

The original and improved sleep-time models were:

$$
T_{wait,old}=0.2N+2.0
$$

$$
T_{wait,new}=0.05N+0.4
$$

For a 5-waypoint path, the waiting time was reduced from approximately `3.00 s → 0.65 s`. Adding U-shaped paths before A* and limiting the number of waypoints also reduced cases where a complex path automatically resulted in a long execution time.

---

## 10. Development Validation

Across 11 real-robot square tests after the planning and execution improvements were integrated, every run achieved a final IoU of at least `0.8`. This was not an isolated ablation of candidate scoring or A* alone; the results include geometry correction, execution-timing improvements, and closed-loop replanning.

<p align="center"><img src="../../images/t1_planning12.png" alt="Task 1 square validation metrics" width="760"/></p>

In the final competition, K-DAS achieved a Task 1 score of `49.69`, ranking 1st. From the planning perspective, the main contributions were:

- evaluating candidate-specific physics predictions and reference-path progress
- including executable approach paths in candidate selection
- correcting model error through observation and replanning after every push

---

## 11. Failure Cases and Limitations

### 11.1 Overshoot and workspace exit

Long strokes are useful for fast motion, but they become risky near the target or workspace boundary. Distance-aware stroke selection and boundary margins are therefore required.

### 11.2 No-plan after retreat

If the retreat distance is too short or mapping error remains, the planner may interpret the gripper and object as overlapping. Increasing the retreat distance can reduce the issue temporarily, but the underlying requirement is consistent observation-to-mapping alignment.

### 11.3 One-step horizon

The current system does not optimize an entire long-horizon push sequence at once. Instead, it uses short-horizon candidate selection with closed-loop replanning.

### 11.4 Discrete action density

Because contact point, direction, and stroke are sampled as discrete candidates, the planner does not guarantee a continuous optimum. Increasing candidate density may improve solution quality, but it also increases the cost of dynamics rollout and path attachment.

---

## 12. Suggested Repository Structure

```text
planning/
├─ README.md
├─ reference_path.py
├─ candidate_generation.py
├─ candidate_scoring.py
├─ approach_path.py
├─ execution_plan.py
├─ configs/
│  ├─ action_space.yaml
│  ├─ scoring_weights.yaml
│  └─ approach_planner.yaml
├─ tests/
│  ├─ test_reference_path.py
│  ├─ test_candidate_scoring.py
│  └─ test_approach_path.py
```

For public release, the following settings should be separated from hard-coded constants.

- contact sampling density
- spin tangent ratio $\lambda$
- distance-based stroke bins
- score weights and penalty thresholds
- obstacle inflation / workspace margin
- A* grid resolution and waypoint limits
- retreat distance and execution sleep

---


> All visual materials are managed in the repository-level `images/` directory.

## 13. Takeaway

The K-DAS Task 1 planner is not a rule-based controller that simply pushes the nearest face toward the target. It is a **model-based manipulation planner that combines a reference path, candidate-wise 1-step physics prediction, multi-objective scoring, safe approach planning, and closed-loop replanning**.

> A good push must not only move the object toward the target. It must also be safely executable by the robot and leave the system in a state that supports the next planning step.
