<p align="center">
  <img src="../../images/t1_runtime01.png" alt="Task 1 runtime execution and recovery pipeline" width="980"/>
</p>

# Runtime Execution for Planar Pushing

**Task 1 · Model-Based Closed-Loop Planar Pushing · Team K-DAS**

This module executes the push action and approach path selected by Planning on the actual CloudGripper system, while handling **observation failures, unreachable approaches, communication latency, insufficient retreat, and prediction error**. Task 1 is not an open-loop system that executes one plan from start to finish. Instead, it uses a closed-loop runtime that performs one push, checks the actual state again, and then generates the next plan.

> Planning determines **what should be executed**, while Runtime is responsible for **whether that plan can be carried out safely and repeatedly on the real robot**.

---

## 1. Role in the Task 1 System

| Layer | Main responsibility |
|---|---|
| Perception / Geometry | estimate object and gripper state, polygon, pose, and IoU |
| Planning | select contact, direction, stroke, and a safe approach path |
| Dynamics | predict the 1-step outcome of each candidate |
| **Runtime** | execute approach, push, retreat, re-observation, and latency/failure recovery |
| Mapping | transform camera pixel coordinates into robot workspace coordinates |

```text
Selected plan
    → observation / state validation
    → approach waypoint execution
    → final alignment
    → push
    → retreat
    → re-observation
    → evaluator / log update
    → next planning step
```

Related documents:

- Task 1 overview: [`../README.md`](../README.md)
- Planning: [`../planning/README.md`](../planning/README.md)
- Dynamics: [`../dynamics/README.md`](../dynamics/README.md)
- Shared Mapping: [`../../mapping/README.md`](../../mapping/README.md)

---

## 2. From Single-step Validation to a Closed Loop

During the initial robot deployment stage, the full loop was not executed immediately. We first verified whether a single push could be planned and executed correctly.

- `task1_one_step_push_eval.ipynb`: Mapping → Perception → Planning → one push
- `task1_loop.ipynb`: re-observe after each push and repeat until the success condition is reached

<p align="center">
  <img src="../../images/t1_runtime02.png" alt="Processed object state before and after one push" width="820"/>
</p>

After single-step validation was completed, the runtime was extended into the following state machine.

<p align="center">
  <img src="../../images/t1_runtime03.png" alt="Closed-loop runtime state machine" width="940"/>
</p>

The key point is that the model-predicted next state is not directly used as the initial state of the next step. After each push, the system reads the actual image and robot state again and reruns the planner from the observed state.

---

## 3. Approach, Push, and Retreat Execution

The runtime executes the waypoints and push segment provided by the planner in the following order.

1. Move through the approach waypoints sequentially at a safe height.
2. Align the gripper with the exact push start position.
3. Execute the push to the planned end point.
4. Retreat in the opposite direction away from the object.
5. Stabilize at a position that does not interfere with the next observation.
6. Acquire a new image and begin the next step.

<p align="center">
  <img src="../../images/t1_runtime04.png" alt="CloudGripper executor output for approach push and retreat" width="700"/>
</p>

```python
for waypoint in approach_path:
    robot.move_xy(*waypoint)
    wait(PATH_SLEEP_TIME)

robot.move_xy(*push_start)
wait(START_SLEEP_TIME)
robot.move_xy(*push_end)
wait(PUSH_SLEEP_TIME)
robot.move_xy(*retreat_point)
wait(RETREAT_SLEEP_TIME)
```

In a public implementation, it would be preferable to verify waypoint completion through robot state feedback and use fixed sleep only for minimal stabilization.

---

## 4. No-plan After Push

During real-robot validation, the first push succeeded, but the planner returned `NO PLAN` on the next step. When the gripper was manually moved backward and the same process was run again, a valid plan was generated. This indicated that the gripper position after execution was a major cause of the failure.

<p align="center">
  <img src="../../images/t1_runtime05.png" alt="No-plan diagnosis and recovery" width="920"/>
</p>

### 4.1 Retreat distance was too short

If the retreat distance is too small, the gripper remains too close to the object in the next observation. The planner may interpret this as a collision or as insufficient free space for motion and reject all approach paths.

As a short-term fix, the retreat distance was increased to provide more planning space for the next step.

### 4.2 Perception and mapping uncertainty

Even when the gripper and object are physically separated, pixel-to-workspace mapping error can make them appear to overlap. The runtime and planner therefore need to distinguish between the following cases.

- penetration into the actual object polygon: reject
- approach path passing through the object: reject
- final contact point lying only inside the inflated safety margin: allow with constraints
- gripper starting inside the safety margin: first move outward to a safe position, then perform normal path planning

<p align="center">
  <img src="../../images/t1_runtime06.png" alt="Task 1 planning geometry in the robot workspace" width="620"/>
</p>

Adjusting the retreat parameter reduces the symptom at runtime, but the underlying requirement is consistent Mapping and accurate object/gripper geometry.

---

## 5. Observation Validation and Recovery

Real observations are not always valid. The gripper can occlude the contour, segmentation can fail temporarily, or a remote image request can be delayed. Passing an invalid observation directly to the planner can produce unsafe actions, so the runtime checks the following conditions.

- both object and gripper detections are available
- polygon area and center are within the workspace bounds
- motion relative to the previous frame is physically plausible
- pose and keypoint ordering are valid
- robot state and visual gripper position do not strongly conflict

```text
valid observation
    → update current state and plan

invalid observation
    → retry image capture
    → move to observation-safe pose if needed
    → keep the previous action unexecuted
    → abort/reset only after repeated failure
```

The recovery principle is to **avoid executing a new push when the current state is uncertain**.

---

## 6. Execution-time Bottleneck Analysis

During intermediate evaluation, a single step took much longer than expected from simulation and local testing. The actual evaluation time included all of the following.

- response time from the overseas evaluation server
- camera image acquisition
- robot state query
- evaluator calls
- network latency
- physical waypoint motion
- fixed sleep inside the code

Early tests required roughly 10 steps to reach the goal, but in the real environment each step was long enough that the run sometimes ended after only 3–4 pushes.

### 6.1 Fixed waiting-time reduction

<p align="center">
  <img src="../../images/t1_runtime07.png" alt="Reduction of fixed waiting times" width="820"/>
</p>

| Waiting item | Before | After |
|---|---:|---:|
| Each approach waypoint | 0.20 s | 0.05 s |
| Arrival at push start | 0.50 s | 0.15 s |
| After push | 1.00 s | 0.10 s |
| After retreat | 0.50 s | 0.15 s |

The original and improved fixed waiting-time models were:

$$
T_{wait,old}=0.2N+2.0
$$

$$
T_{wait,new}=0.05N+0.4
$$

For a 5-waypoint path, sleep time alone was reduced from approximately `3.00 s` to `0.65 s`.

### 6.2 Path-length control

Long A* paths sometimes produced 20 or 31 waypoints, making physical waypoint execution a larger bottleneck than planning computation itself. The following changes were applied.

- Direct → L-shape → U-shape → Short A* priority
- A* waypoint limits
- progressive relaxation of `5 → 7 → 9 → 12`
- removal of unnecessary intermediate waypoints through collision-free shortcutting

---

## 7. Measured Real-robot Runtime

After the improvements, runtime was measured over 10 tests and 43 total push steps using actual terminal timestamps.

<p align="center">
  <img src="../../images/t1_runtime08.png" alt="Measured real robot time per push step" width="860"/>
</p>

- Overall average: `28.02 s / step`
- Evaluation server + vision + robot state: `17.83 s`, or `63.64%` of the total
- Planning: `2.78 s`, or `9.93%`
- Robot motion: `5.94 s`, or `21.21%`
- Controllable planning, motion, wait, and debug section: approximately `10.18 s / step` on average

This analysis showed that improving only planner computation time would have limited impact. Remote I/O latency and unnecessary robot idle time had to be separated and optimized independently. Under the 180-second time limit, the runtime was designed to leave enough execution budget for approximately six pushes.

---

## 8. Runtime Logging and Model Correction

<p align="center">
  <img src="../../images/t1_runtime09.png" alt="Runtime logging schema" width="920"/>
</p>

For each push, the following information is stored together.

- pose, polygon, and IoU before execution
- selected contact, push direction, and stroke
- approach path type and waypoint count
- next pose predicted by the physics model
- next pose actually observed after execution
- prediction residual
- latency and failure reason for each stage

$$
r_t=q_{t+1}^{observed}-\hat q_{t+1}^{physics}
$$

These logs are used to initialize the next planning step from the actual observed state, update the COM belief, and provide data for later residual-correction experiments. Rather than fully trusting or discarding the physics model, the system uses **prediction for action ranking and actual outcomes for the next step and model diagnostics**.

---

## 9. Integrated Runtime Validation

<p align="center">
  <img src="../../images/t1_runtime10.png" alt="Real robot runtime visualization for a circle object" width="640"/>
</p>

Across 11 square development tests with runtime handling, geometry correction, planning, and timing optimization integrated together, every run achieved a final IoU of at least `0.8`. This was not an isolated runtime ablation, but a result of the integrated system configuration.

<p align="center">
  <img src="../../images/t1_runtime11.png" alt="Square object development validation" width="760"/>
</p>

In the final competition, K-DAS achieved a Task 1 score of `49.69`, ranking 1st.

<p align="center">
  <img src="../../images/t1_runtime12.png" alt="Task 1 final ranking" width="720"/>
</p>

---

## 10. Failure Policy

| Failure condition | Runtime response |
|---|---|
| Object or gripper detection missing | capture retry, observation-safe pose |
| No feasible approach path | try next candidate or relaxed path limit |
| Gripper too close after push | increase retreat and re-observe |
| Robot/API command error | stop current step, query state, recover |
| Object outside workspace | do not continue pushing toward boundary; reset if required |
| Evaluator or server latency | preserve state, avoid duplicate command, continue after response |
| Repeated invalid observations | abort current run or controlled reset |

The recovery logic is not intended to keep the robot moving at all costs. It acts as a **fail-safe layer that prevents incorrect pushes when the current state is uncertain**.

---

## 11. Limitations

- Fixed sleep was reduced, but command completion was not handled through a fully event-driven mechanism.
- If Mapping error is large, increasing retreat distance alone cannot fundamentally resolve no-plan failures.
- Remote server latency cannot be directly reduced by the local runtime.
- Runtime logging is useful for diagnosing failures, but isolated ablations for the contribution of each improvement are limited.
- For surprise shapes, contour and contact geometry can differ significantly, so observation-validity checks and approach margins need to remain conservative.

---

## 12. Suggested Repository Structure

```text
runtime/
├─ README.md
├─ executor.py
├─ observation_guard.py
├─ recovery.py
├─ timing.py
├─ logging.py
├─ configs/
│  ├─ execution.yaml
│  └─ recovery.yaml
└─ tests/
   ├─ test_state_machine.py
   ├─ test_no_plan_recovery.py
   └─ test_timing_budget.py
```

Robot tokens, private evaluation URLs, robot-specific calibration values, and absolute paths should be removed from the public configuration.

---

## 13. Takeaway

The K-DAS Task 1 Runtime is not simply a layer that converts planner outputs into API commands. It is the **execution layer that performs approach–push–retreat on the real robot, recovers from observation failures and no-plan conditions, analyzes remote latency, and closes the planning loop again using the actual result at every step**.

> Task 1 reliability was not achieved through physics-model accuracy alone. It was completed by repeatedly combining short-horizon prediction with real re-observation and by avoiding forced execution when the runtime state was uncertain.
