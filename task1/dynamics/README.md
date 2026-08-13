<p align="center">
  <img src="../../images/t1_dynamics01.png" alt="Task 1 rigid pushing dynamics pipeline" width="980"/>
</p>

# Rigid-Body Dynamics for Planar Pushing

**Task 1 · Model-Based Closed-Loop Planar Pushing · Team K-DAS**

Before executing a push candidate on the real robot, this module predicts over a short horizon **how far the object will translate and how much it will rotate**. A simplified Newton-Euler model accounts for object mass, an offset center of mass (COM), moment of inertia, contact location, contact friction, and ground friction. The state is then integrated over time while the pusher moves to estimate the next pose.

> The goal of this model is not to reproduce real contact behavior perfectly, but to quickly compare the relative quality of multiple push candidates generated from the same state. Since the result of each executed action is observed again at every step, model error does not accumulate over a long open-loop horizon.

---

## 1. Role in the Task 1 System

```text
Perception / Geometry
        │
        ▼
Current object pose and boundary
        │
        ▼
Push candidate generation
        │
        ▼
Rigid-body dynamics rollout  ←  object parameter database / COM belief
        │
        ▼
Predicted next pose and polygon
        │
        ▼
Candidate scoring and safe-path feasibility
        │
        ▼
Real push → re-observation → model-belief update
```

The dynamics module does not select the final action by itself. It provides the predicted next state for each candidate, and the planner evaluates that prediction together with reference-path progress, pose error, overshoot risk, and workspace margin.

Related documents:

- Task 1 overview: [`../README.md`](../README.md)
- Geometry and shape alignment: [`../geometry/README.md`](../geometry/README.md)
- Candidate selection: [`../planning/README.md`](../planning/README.md)

---

## 2. Problem Formulation

The planar state of the object is represented by its center position, orientation, linear velocity, and angular velocity.

$$
q_t=[x_t,y_t,\theta_t], \qquad \dot q_t=[v_{x,t},v_{y,t},\omega_t]
$$

A push action consists of the following information:

- contact point or contact ratio along the object boundary
- pusher-center start point
- push direction
- stroke length
- nominal pusher speed

The dynamics module approximates the following transition:

$$
\hat s_{t+1}=f_{\mathrm{rigid}}(s_t,a_t,\phi)
$$

where $\phi$ is a parameter set containing the object mass, center of mass, moment of inertia, friction coefficients, and effective radius.

---

## 3. Modeling Assumptions

To support real-time candidate evaluation, the model was intentionally limited as follows:

- The object is assumed to be a non-deformable planar rigid body.
- The robot pushes the object through 2D pusher motion without Z-axis manipulation or grasping.
- Pusher-object contact is approximated as a point contact with a local normal and tangent.
- Contact resistance and ground resistance are represented using a simplified Coulomb-friction model.
- Each push is divided into short time intervals and forward-integrated.
- **Short-horizon trends needed for candidate ranking** are prioritized over long-horizon accuracy.

This is not a full limit-surface identification model or a high-resolution contact-mechanics simulator. It is a grey-box simulator designed to prioritize computational efficiency and physically meaningful trends when repeatedly evaluating dozens of candidates in a remote robot environment.

---

## 4. Object Mass, Center of Mass, and Inertia

<p align="center">
  <img src="../../images/t1_dynamics02.png" alt="Mass center and inertia modeling" width="900"/>
</p>

For the Weighted Square and Weighted Circle, the geometric center and center of mass can differ depending on the internal weight configuration. The total mass and COM are computed as

$$
M=\sum_i m_i, \qquad c=\frac{\sum_i m_i r_i}{\sum_i m_i}
$$

The moment of inertia about the new COM is computed using the parallel-axis theorem.

$$
I=\sum_i\left(I_i+m_i\lVert r_i-c\rVert^2\right)
$$

For the T-shape, the top bar and stem are treated as separate rectangles, and the mass, center, and moment of inertia of each part are combined.

### Why COM matters

If the lever arm from the contact point to the COM changes, the same force can produce a different torque.

$$
r=p_{contact}-c
$$

$$
\tau_{push}=r_xF_y-r_yF_x
$$

Therefore, using only the geometric center makes it difficult to correctly compare the direction and magnitude of rotation for objects with asymmetric mass distributions.

---

## 5. Contact Force and Slip Approximation

<p align="center">
  <img src="../../images/t1_dynamics03.png" alt="Contact force decomposition and slip model" width="900"/>
</p>

The push force is decomposed using the outward normal $\hat n$ and tangent $\hat t$ at the contact surface.

$$
F_n=|F_{push}\cdot\hat n|, \qquad F_t=F_{push}\cdot\hat t
$$

If the tangential component is below the contact-friction limit, the contact is treated as sticking and the commanded force is transmitted.

$$
|F_t|\le\mu_cF_n \quad\Rightarrow\quad F_{app}=F_{push}
$$

If the limit is exceeded, the tangential component is capped at the friction limit.

$$
|F_t|>\mu_cF_n
$$

This branching reduces cases in which a candidate that only glances across the object at an angle would otherwise be predicted to transfer unrealistically large force.

---

## 6. Ground Friction and Newton-Euler Equations

Translational friction is applied opposite to the object's current linear velocity.

$$
F_{fric}=-\frac{v}{\lVert v\rVert}\mu_gmg
$$

For rotation, a resisting torque is applied using an effective friction radius $R_{eff}$.

$$
\tau_{fric}=-\mathrm{sgn}(\omega)\mu_gmgR_{eff}
$$

The resulting linear and angular accelerations are

$$
a=\frac{F_{app}+F_{fric}}{m}
$$

$$
\alpha=\frac{\tau_{push}+\tau_{fric}}{I}
$$

This structure directly captures the basic physical trend that rotation is small when the push-force line passes through the COM, while a larger offset from the COM produces more torque and therefore a stronger rotational effect.

---

## 7. Time Integration and Moving Contact

<p align="center">
  <img src="../../images/t1_dynamics04.png" alt="Moving contact rollout" width="900"/>
</p>

The initial model divided each stroke into fixed distance increments and assumed a fixed contact point. On the real robot, however, the pusher moves from the start point to the end point at approximately constant speed. The final model therefore updates both push duration and contact position over time.

### 7.1 Measured robot speed

The actual pushing speed of the robot was measured over 15 trials, each with a commanded travel distance of **0.09 m**. The overall mean pushing speed was approximately **0.0933 m/s**.

| Measurement Set | Trial | Travel Distance (m) | Travel Time (s) | Speed (m/s) |
|---|---:|---:|---:|---:|
| 1st | 1 | 0.0900 | 0.9767 | 0.0921 |
| 1st | 2 | 0.0900 | 0.9633 | 0.0934 |
| 1st | 3 | 0.0900 | 0.9195 | 0.0979 |
| 1st | 4 | 0.0900 | 0.9618 | 0.0936 |
| 1st | 5 | 0.0900 | 0.9315 | 0.0966 |
| **1st Mean** | — | — | — | **0.0947** |
| 2nd | 1 | 0.0900 | 0.9686 | 0.0929 |
| 2nd | 2 | 0.0900 | 0.9515 | 0.0946 |
| 2nd | 3 | 0.0900 | 0.9607 | 0.0937 |
| 2nd | 4 | 0.0900 | 1.0006 | 0.0899 |
| 2nd | 5 | 0.0900 | 0.9906 | 0.0909 |
| **2nd Mean** | — | — | — | **0.0924** |
| 3rd | 1 | 0.0900 | 0.9455 | 0.0952 |
| 3rd | 2 | 0.0900 | 0.9965 | 0.0903 |
| 3rd | 3 | 0.0900 | 0.9534 | 0.0944 |
| 3rd | 4 | 0.0900 | 0.9343 | 0.0963 |
| 3rd | 5 | 0.0900 | 1.0267 | 0.0877 |
| **3rd Mean** | — | — | — | **0.0928** |
| **Overall Mean** | **15 trials** | — | — | **0.0933** |

The measured mean speed was used to convert each push stroke length into a rollout duration.

$$
T=\frac{L}{v_{push}}, \qquad N=\left\lceil\frac{T}{\Delta t}\right\rceil
$$

### 7.2 Moving contact point

At integration step $k$, the contact point moves along the push line.

$$
\alpha_k=\frac{k+1}{N}
$$

$$
p_c(k)=p_s+\alpha_k(p_e-p_s)
$$

The lever arm and torque are therefore recomputed at every step.

$$
r(k)=p_c(k)-c, \qquad \tau(k)=r(k)\times F(k)
$$

### 7.3 State update

$$
v_{k+1}=v_k+a_k\Delta t
$$

$$
\omega_{k+1}=\omega_k+\alpha_k\Delta t
$$

$$
q_{k+1}=q_k+[v_{k+1,x},v_{k+1,y},\omega_{k+1}]\Delta t
$$

---

## 8. Pusher Radius and the True Start Condition

In the initial candidate-generation method, the pusher-center start point was placed directly on the object surface. Because the real pusher has a finite radius, however, moving the pusher center to that point would already cause contact and push the object during the approach.

The start condition was therefore corrected as follows.

$$
p_{start}=p_{surface}+R_{pusher}\hat n
$$

<p align="center">
  <img src="../../images/t1_dynamics06.png" alt="Pusher radius start point correction" width="800"/>
</p>

This sim-to-real correction ensures that the simulator and the real robot use the same contact-start condition.

---

## 9. Shape-Specific Contact Geometry

### Circle

For a circle, the local normal is defined by the radial direction from the geometric center to the contact point.

$$
\hat n=\frac{p_c-p_{geo}}{R}
$$

Even when the COM is offset, the surface normal is computed from the geometric center of the object boundary.

### Polygonal objects

For the Square and T-shape, the tangent is computed from the current polygon edge, and the outward normal is taken perpendicular to that tangent.

$$
\hat t=\frac{p_2-p_1}{\lVert p_2-p_1\rVert}, \qquad
\hat n=[t_y,-t_x]
$$

In the initial prototype, axis-aligned local normals were defined case by case. In the integrated planner, this was extended to construct contact geometry from the currently detected boundary and orientation.

---

## 10. Candidate-Wise 1-Step Prediction

Each candidate is passed through the same dynamics function to generate a predicted next pose.

```python
for candidate in candidates:
    state = current_state.copy()
    duration = candidate.stroke / measured_push_speed
    steps = ceil(duration / dt)

    for k in range(steps):
        contact = interpolate(candidate.contact_start,
                              candidate.contact_end,
                              (k + 1) / steps)
        force = resolve_contact_force(candidate.force,
                                      candidate.normal,
                                      candidate.tangent)
        acceleration, angular_acceleration = newton_euler(
            state, force, contact, object_parameters
        )
        state = integrate(state, acceleration,
                          angular_acceleration, dt)

    candidate.predicted_pose = state.pose
```

The dynamics output provides the following information:

- predicted center and orientation
- predicted polygon / keypoints
- translation and rotation magnitude
- predicted progress toward a reference pose
- diagnostics such as applied torque and slip state

The planner uses this output as part of the candidate score.

---

## 11. CAD Candidate Cases and Online COM Belief

The internal weight location of the real object could not be observed directly in advance. Therefore, mass, inertia, and COM candidates were precomputed in CAD for each feasible nut configuration and stored in a database.

After each real push, the observed translational and rotational response is compared with the prediction from each candidate model.

$$
e_j=\lVert\Delta p_{obs}-\Delta p_j\rVert
+w_\theta|\Delta\theta_{obs}-\Delta\theta_j|
$$

The belief of cases with lower error is increased, or the best-matching case is used as the parameter set for the next planning step.

<p align="center">
  <img src="../../images/t1_dynamics07.png" alt="Closed-loop COM belief update" width="900"/>
</p>

The key idea is not to estimate the object's mass distribution once and keep it fixed, but to **use the actual push response to guide the next model selection**.

---

## 12. Physics Validation

### 12.1 Symmetric vs. asymmetric mass distribution

<p align="center">
  <img src="../../images/t1_dynamics08.png" alt="Symmetric and asymmetric dynamics validation" width="850"/>
</p>

The following trends were observed in a 1.0 s simulation.

| Case | Object / COM | Push condition | Final behavior |
|---|---|---|---|
| 1 | Square, centered COM | Bottom-center upward | Pure translation |
| 2 | Square, lower-right COM | Same push | Translation + clockwise rotation |
| 3 | Square, upper-left COM | Same push | Translation + counter-clockwise rotation |
| 4 | Circle, centered COM | Bottom-center upward | Pure translation |
| 5 | Circle, lower-right COM | Same push | Translation + clockwise rotation |

With a centered COM, the force line passes through the COM, so $\omega$ and $\theta$ remain close to zero. Reversing the direction of the COM offset also reverses the sign of rotation.

### 12.2 Contact location and asymmetric shape

<p align="center">
  <img src="../../images/t1_dynamics09.png" alt="Contact location and T-shape validation" width="850"/>
</p>

Even for the same asymmetric circle, changing the contact location and push direction changes the axis-wise components of translational velocity. For the T-shape, a stem-center push whose force line passes through the COM produces nearly pure translation, while contact near the side of the head produces both substantial translation and rotation.

This validation is not a calibration test intended to prove exact numerical agreement with the real hardware. It is a sanity check to verify that the model reproduces the **directional physical trends** needed for candidate ranking.

---

## 13. Real-Robot Validation and Sim-to-Real Correction

### 13.1 Square development tests

- Mean IoU over 9 random-target tests: **0.7636**
- Mean number of steps: **7.2**
- Final IoU in a single test using an actual competition target: **0.8026**
- Test under competition-like conditions: 5 steps, final IoU **0.8**

### 13.2 Circle development tests

<p align="center">
  <img src="../../images/t1_dynamics10.png" alt="Circle closed-loop development results" width="820"/>
</p>

Across 5 tests using the official target, the mean final IoU was **0.90**, the mean number of steps was **6.8**, and the mean time was **116.9 s**.

<p align="center">
  <img src="../../images/t1_dynamics11.png" alt="Official circle evaluation result" width="800"/>
</p>

In the competition configuration with visualization removed, the system achieved a final IoU of **1.0** and an official evaluation score of **67.1**.

These values correspond to development validation in week 6 and should be distinguished from the final overall Task 1 competition score. The results also reflect the combined system performance of perception, candidate selection, path planning, and closed-loop re-observation, rather than the dynamics model alone.

---

## 14. Why Closed-Loop Replanning Was Essential

Real-world pushing contains several sources of error at the same time:

- unknown floor friction and contact friction
- pusher-center detection error
- object contour and pose estimation noise
- robot-cell calibration differences
- unmodeled compliance and transient contact
- mass-distribution mismatch

For this reason, the physics model was not used as a long-horizon open-loop controller. After each real action, the new pose is observed again, the difference between prediction and observation is recorded, and a new candidate set is generated from the updated state.

```text
Predict one push
    ↓
Execute one push
    ↓
Observe actual result
    ↓
Update current state / COM belief
    ↓
Replan from the new state
```

This makes the dynamics model a short-horizon physical prior inside a closed-loop planner rather than a **perfect digital twin**.

---

## 15. Limitations

### 15.1 Simplified friction

Because the model uses a simplified Coulomb-friction coefficient and effective rotation radius, it does not directly identify distributed pressure or the full limit surface.

### 15.2 Point-contact approximation

The real pusher has a finite contact patch, but the model approximates it as a local point contact.

### 15.3 Parameter identifiability

It is difficult to uniquely identify COM, floor friction, and contact friction at the same time using only observed pose changes. Different parameter combinations can produce similar responses.

### 15.4 Concave / unseen shapes

For concave surprise shapes such as Plus and Organic, local contact geometry and rotational response are more complex, which can increase error when using rigid templates and simple contact sampling.

### 15.5 Learned correction was not the final deployed core

During later development, physics predictions and actual results were stored to collect data for a residual-correction model. However, the core of the final Task 1 pipeline remained physics rollout and closed-loop replanning. This document therefore does not present the learned correction model as a fully deployed final component.

---

## 16. Suggested Repository Structure

```text
dynamics/
├─ README.md
├─ src/
│  ├─ rigid_body.py          # Newton-Euler state update
│  ├─ contact_model.py       # normal/tangent and slip handling
│  ├─ object_database.py     # CAD mass / COM / inertia candidates
│  ├─ rollout.py             # moving-contact one-push simulation
│  └─ belief_update.py       # predicted-observed case matching
├─ configs/
│  ├─ objects.yaml
│  ├─ friction.yaml
│  └─ robot_motion.yaml
├─ tests/
│  ├─ test_symmetric_push.py
│  ├─ test_com_sign.py
│  └─ test_moving_contact.py
```

For the public repository, the following parameters are appropriate to separate into config files:

- measured pusher speed and pusher radius
- integration timestep
- contact and ground friction coefficients
- object mass / COM / inertia candidates
- effective friction radius
- maximum stroke and near-goal stroke bins

---

> All visual assets are managed in the repository-level `images/` directory.

## 17. Takeaway

The K-DAS Task 1 dynamics module is not a high-fidelity simulator intended to reproduce complex planar pushing contact perfectly. Instead, it is a 1-step physical predictor that incorporates **center of mass, moment of inertia, contact location, force and torque, friction, measured pusher speed, and moving contact** to quickly compare the next states of multiple candidates.

> The core idea of this module is to **use the physics model to predict the direction of the next action, then correct the plan at every step using the actual execution result**.
