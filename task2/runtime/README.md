<img src="../../images/t2_runtime01.png" alt="Geometry-aware runtime safety layer" width="950"/>

# Runtime Safety for Real-Robot Rope Manipulation

**Task 2 · Linear Deformable Object Shape Control · Team K-DAS**

이 모듈은 BC/RL policy가 출력한 action을 실제 CloudGripper에서 실행하기 전에 검증하고, 필요한 경우 제한·보정하며, 높은 score 상태를 마지막 평가 시점까지 유지하기 위한 **geometry-aware runtime safety layer**를 설명한다.

Runtime safety는 학습 모델을 대체하는 별도의 policy가 아니다. Policy가 선택한 node·drag length·direction을 그대로 실행했을 때 발생할 수 있는 실제 하드웨어 불확실성을 다루는 실행 계층이다.

> **핵심 질문**  
> “학습된 policy는 유지하면서, 실제 로봇의 회전 방향·servo 범위·endpoint 민감도·release 이후 형상 붕괴를 어떻게 줄일 수 있는가?”

---

## 1. Why a Runtime Safety Layer Was Needed

DER, Residual GNN, MPC teacher, BC/RL을 거치면서 action 선택 성능은 개선되었지만, 실제 로봇에서는 simulation이나 offline rollout에 없던 문제가 반복적으로 나타났다.

- Rope detection이 일시적으로 실패하여 잘못된 state가 입력되는 문제
- Grasp 이후 local tangent가 목표와 반대로 정렬되는 문제
- 초기 step에서 endpoint를 먼저 조작해 전체 형상이 불안정해지는 문제
- `robot.rotate(angle)`과 실제 로프 회전 방향이 직관과 다르게 작동하는 문제
- Servo의 실제 절대각 범위가 `0°–180°`로 제한되는 문제
- Node 18·19에서 회전 전달이 중앙부보다 불안정한 문제
- Gripper release 직후 로프가 이완되어 이미 확보한 score가 감소하는 문제
- Reset 이후 로프가 특정 방향으로 기울어진 상태에서 시작하는 문제

따라서 최종 시스템은 다음 구조로 발전했다.

```text
Learned policy
      │
      ▼
Geometry-aware action validation
      │
      ▼
Real-robot execution and score preservation
```

관련 문서:

- Task 2 overview: [`../README.md`](../README.md)
- MPC teacher: [`../planning/mpc/README.md`](../planning/mpc/README.md)
- Candidate-aware RL: [`../policy/candidate_aware_rl/README.md`](../policy/candidate_aware_rl/README.md)

---

## 2. Safety Layer Overview

Runtime safety는 크게 다섯 역할을 수행한다.

| Stage | Role | Main mechanism |
|---|---|---|
| Observation safety | 잘못된 state로 action을 실행하지 않음 | Rope detection validation, automatic reset |
| Initial stabilization | 초기 형상 전체를 먼저 안정화 | Central-node priority, target-directed initial push |
| Direction safety | Local direction이 반대로 정렬되는 것을 방지 | Directed tangent, signed area, rotation probe |
| Hardware safety | 불가능하거나 과도한 회전을 제한 | Servo-feasible angle, geometry risk, endpoint limits |
| Score preservation | 좋은 형상을 마지막 평가까지 유지 | Step-level hold, pre-release score hold |

---

## 3. Observation Validation and Automatic Recovery

Rope node extraction에 실패하거나 node ordering이 깨진 상태에서 policy action을 실행하면, 이후 모든 geometry 계산이 무의미해질 수 있다. 따라서 rope observation이 유효하지 않으면 action을 강행하지 않고 다음 순서로 복구한다.

1. Rope node 개수와 ordering 유효성 확인
2. Current/target coordinate 범위 확인
3. 연속 detection failure count 증가
4. 설정된 횟수 이상 실패하면 environment reset
5. Reset 이후 state를 다시 관측한 뒤 planning 재개

이 구조는 일시적인 조명 변화나 segmentation failure가 전체 run 실패로 이어지는 것을 줄였다.

---

## 4. Initial Stabilization Strategy

### 4.1 Central-node priority

초기 1–2 step에서는 endpoint보다 중앙부 node를 우선 선택하였다. 중앙부를 먼저 조작하면 로프 중심 위치와 전체 방향을 안정화한 뒤 endpoint를 세부 조정할 수 있다. 반대로 endpoint부터 크게 이동시키면 작은 오차가 전체 형상으로 전파될 수 있다.

### 4.2 Target-directed initial push

Reset 이후 로프가 완전히 중립적으로 놓이지 않고 한쪽으로 기울어지는 bias가 관찰되었다. 이를 줄이기 위해 target 방향을 기준으로 node 19 부근을 반대 방향으로 한 번 push하였다.

| Target direction | Initial correction | Purpose |
|---|---|---|
| Target이 위쪽 | Node 19를 아래쪽으로 push | 위쪽 목표로 변형될 여유 확보 |
| Target이 아래쪽 | Node 19를 위쪽으로 push | 아래쪽 목표에서 endpoint 이탈 방지 |
| Neutral / unknown | Fallback sign 사용 | 불확실한 경우 기본 보정 적용 |


---

## 5. Node-Order-Based Directed Tangent

Rope는 단순한 점 집합이 아니라 순서를 가진 선형 구조다. 따라서 선택 node의 위치가 목표에 가까워도, local tangent의 방향이 반대로 정렬되면 전체 형상은 잘못될 수 있다.

선택 node `i` 주변에서 node index 증가 방향의 tangent를 다음처럼 정의하였다.

$$
 t_i = \frac{p_{i+k}-p_{i-k}}{\|p_{i+k}-p_{i-k}\|}
$$

현재 rope와 target rope의 tangent를 각각 `t_cur`, `t_goal`이라 하면 signed angle은 다음과 같다.

$$
\Delta_{dir}=\operatorname{atan2}(t_{cur,x}t_{goal,y}-t_{cur,y}t_{goal,x},\ t_{cur}\cdot t_{goal})
$$

`directed_delta`는 단순한 angle difference가 아니라 **node ordering을 유지한 상태에서 어느 방향으로 회전해야 하는지**를 나타낸다.

<img src="../../images/t2_runtime02.png" alt="Directed tangent and signed area concept" width="800"/>

---

## 6. Signed Area and Bend Diagnosis

선택 node 주변 세 점을 사용하여 local bend direction을 진단하였다.

$$
 a=p_i-p_{i-k},\qquad b=p_{i+k}-p_i
$$

$$
 A=a_xb_y-a_yb_x
$$

현재와 목표의 signed area 부호를 비교하면 local bend가 같은 방향인지 확인할 수 있다.

| Diagnosis | Meaning | Caution |
|---|---|---|
| `bend_same_sign = True` | 현재와 목표가 같은 방향으로 휘어 있음 | 선택 action이 안전하다는 뜻은 아님 |
| `False` | 반대 방향으로 휘어 있음 | Direction correction 후보가 될 수 있음 |
| `None` | Signed area가 0에 가까움 | Endpoint에서는 판단 신뢰도가 낮음 |

Signed area는 보조 지표이며, 최종 안전성은 directed delta, selected rotation, overshoot, reversal, endpoint 여부와 함께 판단한다.

---

## 7. Rotation Probe and Command Sign Correction

초기에는 selected rotation과 directed delta의 부호가 같으면 올바른 회전이라고 가정했다. 그러나 robot27에서 rotation만 단독으로 수행한 probe 결과, `robot.rotate(+δ)`는 directed delta를 감소시키기보다 대체로 증가시키는 방향으로 작동했다.

따라서 directed delta를 0으로 줄이기 위한 desired command sign은 다음처럼 해석했다.

$$
\operatorname{sign}(\Delta_{cmd,desired})\approx-\operatorname{sign}(\Delta_{dir})
$$

이 결과를 모든 action에 무조건 반전 적용한 것은 아니다. 현재 selected action이 실측 sign convention과 충돌하는 경우에만 partial direction correction을 적용했다.

<img src="../../images/t2_runtime03.png" alt="Rotation probe scatter plot" width="800"/>

Node 8–17에서는 command와 directed delta 변화의 correlation이 모두 0.995 이상으로 높았다. 반면 node 19에서는 correlation이 0.663으로 낮았다.

<img src="../../images/t2_runtime04.png" alt="Rotation probe slope and correlation by node" width="800"/>

| Node | Slope | Correlation | Interpretation |
|---:|---:|---:|---|
| 8 | +0.739 | 0.995 | 회전 명령이 local direction 변화에 일관되게 반영 |
| 10 | +0.733 | 0.997 | 높은 선형 일관성 |
| 12 | +0.808 | 0.996 | 높은 선형 일관성 |
| 14 | +0.918 | 0.995 | 높은 선형 일관성 |
| 17 | +1.115 | 0.997 | Endpoint 근처이나 비교적 일관됨 |
| 19 | +0.235 | 0.663 | Endpoint 특성으로 회전 전달 불안정 |

---

## 8. Geometry Risk Control

Geometry risk는 최종 task score가 아니라, **선택된 rotation이 local geometry와 충돌할 가능성**을 진단하는 보조 지표다.

평가 요소는 다음과 같다.

| Risk factor | Check | Control response |
|---|---|---|
| Direction sign consistency | Selected delta가 `-sign(directed_delta)`와 일치하는가 | 불일치 시 correction 또는 risk 증가 |
| Magnitude risk | 회전량이 과도한가 | Risk scale로 회전량 감쇠 |
| Overshoot risk | 필요한 correction보다 더 멀리 회전하는가 | HIGH/VERY_HIGH로 승격 |
| Direction reversal | Directed delta가 ±150° 부근인가 | Local 방향 반전 상태로 판단 |
| Endpoint risk | Node 18·19에서 area 신뢰도가 낮은가 | Endpoint 전용 limit 적용 |

Risk가 높을수록 다음 대응을 적용한다.

- 원래 action 유지
- Rotation magnitude 감쇠
- 제한된 partial correction
- Alternative servo target 선택
- Rotation 생략

Geometry risk가 HIGH라고 해서 반드시 action 결과가 나쁜 것은 아니다. Translation 효과가 충분히 큰 경우 HIGH risk action에서도 전체 node error가 감소할 수 있으므로, risk는 실패 판정이 아니라 **보수적 실행을 위한 진단 신호**로 사용한다.

---

## 9. Servo-Feasible Angle Selection

CloudGripper servo는 실제로 `0°–180°` 절대각 범위에서 동작한다. 따라서 360° wrap을 가정한 단순 angle difference는 불가능하거나 과도한 회전을 만들 수 있다.

목표 local tangent의 축 동치성을 고려하여 다음 후보를 평가한다.

```text
goal
goal + 180°
goal - 180°
```

각 후보에 대해 다음 조건을 확인한다.

1. Servo target이 `0°–180°` 범위 안에 있는가
2. 현재 angle에서의 delta가 최대 허용량 이내인가
3. Geometry risk가 허용 가능한가

실행 가능한 후보가 없거나 최대 delta를 넘으면 after-grasp rotation을 생략한다. 이를 통해 `current=168.11°`, `goal=11.31°`와 같은 0/180 경계의 비정상적 대회전을 줄였다.

---

## 10. Endpoint-Specific Correction

Endpoint는 전체 형상에 큰 영향을 주지만, 접촉력과 장력 전달이 중앙부보다 불안정하다. 특히 node 19의 rotation probe correlation이 낮았기 때문에 중앙부와 동일한 correction을 적용하지 않았다.

Endpoint에서는 다음 설정을 별도로 둔다.

- Maximum rotation delta
- Direction correction ratio
- Geometry risk scale
- Signed-area 신뢰도 축소

즉 endpoint correction을 금지한 것이 아니라, **필요한 보정은 허용하되 중앙부보다 보수적으로 제한**하였다.

---

## 11. Score Hold and Pre-Release Hold

Task 2 평가는 제한 시간 중 최고 score가 아니라 마지막 시점의 score가 중요할 수 있다. 따라서 높은 score에 도달한 이후에도 계속 action을 수행하는 것이 항상 유리하지 않다.

### 11.1 Time-dependent hold threshold

| Remaining time | Hold threshold | Purpose |
|---:|---:|---|
| 60초 이상 | 0.90 | 충분히 높은 점수에서만 조기 hold |
| 30–60초 | 0.80 | 후반부 불필요한 추가 action 감소 |
| 30초 미만 | 0.70 | 종료 직전 score 손실 방지 |

### 11.2 Pre-release score hold

기존 실행:

```text
grasp → rotate → drag → gripper open → observe
```

수정된 실행:

```text
grasp → rotate → drag
      → gripper open 전에 eval_status 확인
      → score가 threshold 이상이면 closed hold
      → score 유지 여부를 재확인
      → 유지되면 hold, 하락하면 release / replan
```

<img src="../../images/t2_runtime05.png" alt="Pre-release score hold flow" width="850"/>

이 전략의 목적은 drag 결과가 좋더라도 release 순간 rope가 튀거나 이완되어 score가 감소하는 것을 막는 것이다.

---

## 12. Real-Robot Execution Example

아래 예시는 실제 실행에서 current rope가 target shape에 가까워지는 과정을 보여준다. Runtime safety layer는 policy action을 새로 생성한 것이 아니라, action이 실제 하드웨어에서 안정적으로 전달되도록 direction과 execution 조건을 보정하였다.

<img src="../../images/t2_runtime06.png" alt="Real robot state before and after an action" width="850"/>

---

## 13. Quantitative Runtime Results

<img src="../../images/t2_runtime07.png" alt="Runtime error reduction from robot logs" width="850"/>

확인 가능한 실제 로그를 분석한 결과는 다음과 같다.

| Log set | Valid actions | Improved actions | Improvement ratio | Mean error before | Mean error after | Mean reduction |
|---|---:|---:|---:|---:|---:|---:|
| 전체 확인 가능 로그 | 18 | 14 | 77.8% | 173.69 mm | 106.23 mm | 67.46 mm |
| Robot11 개선 로그 | 11 | 10 | 90.9% | 168.18 mm | 72.00 mm | 96.18 mm |

이 결과는 실제 실행 action이 전반적으로 목표 형상에 가까워지는 방향으로 작동했음을 보여준다. 다만 이 로그만으로 policy와 runtime safety 각각의 기여도를 분리할 수는 없다. 해당 분리를 위해서는 동일 policy에 대해 safety layer ON/OFF ablation이 필요하다.

---

## 14. Execution Pseudocode

```python
def execute_with_runtime_safety(current, target, policy_action, remaining_time):
    if not valid_rope_observation(current):
        return recover_or_reset()

    action = apply_initial_schedule(policy_action, current, target)

    directed_delta = compute_directed_tangent_error(current, target, action.node)
    bend_sign = compute_signed_area_diagnosis(current, target, action.node)

    desired_sign = -sign(directed_delta)
    risk = evaluate_geometry_risk(
        selected_delta=action.rotation,
        desired_sign=desired_sign,
        directed_delta=directed_delta,
        bend_sign=bend_sign,
        endpoint=is_endpoint(action.node),
    )

    action = choose_servo_feasible_angle(action, risk)
    action = apply_endpoint_limits(action)

    robot.grasp(action.node)
    robot.rotate(action.rotation)
    robot.drag(action.target)

    score = read_eval_status()
    threshold = hold_threshold(remaining_time)

    if score >= threshold:
        keep_gripper_closed()
        if score_is_stable():
            return HOLD

    robot.gripper_open()
    log_transition_and_reason()
    return CONTINUE
```

---

## 15. Limitations

### 15.1 Rotation and drag effects are not fully separated

Rotation probe는 rotation만 단독 수행한 실험이다. 실제 action에서는 rotate와 drag가 연속으로 작동하므로, local tangent 변화가 어느 동작에서 발생했는지 분리하기 어렵다.

### 15.2 Endpoint correction remains uncertain

Node 19 correlation이 낮으므로 endpoint rotation을 지나치게 신뢰하면 불안정한 변형이 발생할 수 있다.

### 15.3 Initial push contact is not guaranteed

Initial push 효과는 gripper open/close 상태, z-height, workspace 위치에 따라 달라질 수 있다.

### 15.4 Hold threshold requires further ablation

Hold 중 score가 순간적으로 상승했다가 다시 하락할 수 있으므로, threshold와 release condition을 더 체계적으로 비교해야 한다.

### 15.5 Failure samples are sparse

낮은 빈도의 실패를 충분히 분류하려면 reason별 JSONL/CSV 로그를 지속적으로 축적해야 한다.

---

## 16. Recommended Repository Structure

```text
runtime_safety/
├─ README.md
├─ execution/
│  ├─ observation_guard.py
│  ├─ action_executor.py
│  └─ recovery.py
├─ geometry/
│  ├─ directed_tangent.py
│  ├─ signed_area.py
│  ├─ rotation_probe.py
│  └─ geometry_risk.py
├─ constraints/
│  ├─ servo_feasible_angle.py
│  ├─ endpoint_limits.py
│  └─ workspace_guard.py
├─ score_hold/
│  ├─ hold_policy.py
│  └─ pre_release_hold.py
├─ configs/
│  ├─ runtime_safety.yaml
│  └─ robot_overrides.yaml
├─ logs/
│  └─ README.md
```

Robot-specific sign convention과 endpoint limit은 일반 runtime logic과 분리하여 `robot_overrides.yaml`에서 관리하는 것이 적절하다.

---


> 모든 시각 자료는 repository 최상위 `images/` 폴더에서 공통 관리한다.

## 17. Takeaway

K-DAS의 Runtime Safety는 학습된 policy 위에 단순 threshold를 추가한 구조가 아니다. Node ordering을 반영한 directed tangent, 실측 rotation probe, servo-feasible angle, geometry risk, endpoint-specific limit, initial push, pre-release score hold를 결합한 **실제 로봇 실행 안정화 계층**이다.

> **Policy가 좋은 action을 제안하고, runtime safety가 그 action을 실제 하드웨어에서 실행 가능하고 보존 가능한 형태로 만든다.**
