<p align="center">
  <img src="../images/t2_overview01.png" alt="Task 2 rope manipulation" width="920"/>
</p>

# Task 2 — Linear Deformable Object Shape Control

**IEEE ICRA 2026 Cloud Robotics Competition · Team K-DAS · Kookmin University**

Task 2의 목표는 한쪽 끝이 고정된 rope-like deformable object를 조작하여, 제한 시간의 마지막 시점에 목표 형상과 최대한 유사하게 만드는 것이다. 강체의 위치와 자세를 맞추는 Task 1과 달리, Task 2에서는 로프 전체의 곡선 형상, 노드 순서, 국소 접선 방향, 길이 일관성을 함께 고려해야 한다.

K-DAS는 로프를 **20개의 ordered node**로 표현하고, **DER 기반 물리 모델**, **Residual GNN**, **2-step MPC**, **Behavior Cloning / candidate-aware offline RL**, 그리고 **geometry-aware runtime safety**를 연결한 폐루프 조작 시스템을 개발하였다.

> 이 문서는 Task 2 전체의 문제 정의, 개발 흐름, 최종 시스템과 성과를 설명하는 상위 README이다. 수식, 모델 구조, 학습 설정, 세부 ablation과 실행 코드는 각 모듈 문서에서 별도로 다룬다.

---

## 1. Competition Result

| 항목 | K-DAS 결과 |
|---|---:|
| Task 2 Final Score | **81.64** |
| Original Rope | **92.61** |
| Longer Rope | **87.73** |
| Thicker Red Rope | **64.59** |
| Mid-term Evaluation | **76.90 / 100, 1st place** |
| Competition Overall | **1st place** |

Original rope와 longer rope에서 높은 성능을 기록한 것은 20-node representation, target-edge 기반 예측, 2-step teacher와 폐루프 재계획이 로프 길이 변화에 대해 비교적 안정적으로 작동했음을 보여준다. 반면 thicker red rope에서는 색상, 두께, 굽힘 강성, 마찰, gripper contact 특성이 달라지면서 perception과 dynamics 양쪽에서 domain gap이 증가하였다.

<p align="center">
  <img src="../images/t2_overview02.png" alt="Task 2 final ranking" width="820"/>
</p>

---

## 2. System at a Glance

<img src="../images/t2_overview03.png" alt="Task 2 pipeline" width="900"/>

```text
Top-view image
    ↓
Rope segmentation and ordered 20-node state
    ↓
Shared pixel-to-workspace mapping
    ↓
DER physical rollout
    ↓
Residual GNN correction + target-edge consistency
    ↓
2-step MPC teacher / learned BC-RL policy
    ↓
Geometry-aware runtime correction
    ↓
Grasp → rotate → drag → score check → release or hold
    ↓
Re-observe and re-plan
```

최종 시스템은 한 번의 action으로 목표 형상을 완성하려 하지 않는다. 매 step마다 현재 로프를 다시 관측하고, action을 선택·실행한 뒤, 실제로 바뀐 형상에서 다음 계획을 다시 계산한다. 이 폐루프 구조는 dynamics model과 실제 로봇 사이의 오차가 누적되는 것을 줄이고, 높은 점수에 도달한 상태를 유지하는 데 핵심적인 역할을 한다.

---

## 3. Development Story

### 3.1 로프 형상을 무엇으로 표현할 것인가?

> “강체처럼 중심 위치와 회전각만 맞추면 되는가?”

로프는 단일 pose로 표현할 수 없다. 같은 중심과 방향을 가지더라도 중간 부분의 굽힘과 말단 형상이 다르면 전혀 다른 상태가 된다. 우리는 관측된 로프 형상을 진행 순서를 유지하는 **20개 ordered node**로 변환하고, 현재 node와 목표 node의 대응 관계를 기준으로 제어 문제를 정의하였다.

개발 과정에서는 다음 지표를 함께 사용하였다.

- node-wise mean error와 RMSE
- 인접 node 사이의 edge length consistency
- 전체 형상 및 최종 evaluation score
- local tangent와 endpoint 방향성


---

### 3.2 물리 모델만으로 로프의 다음 형상을 예측할 수 있는가?

> “어떤 node를 잡아 어느 방향으로 옮기면, 로프 전체는 어떻게 변할 것인가?”

좋은 action을 선택하려면 action 이후의 로프 형상을 미리 예측할 수 있어야 한다. 초기에는 Discrete Elastic Rods의 관점을 참고하여 로프를 node-edge chain으로 모델링하였다. 모델에는 길이 제약, 굽힘 저항, damping, 고정단, grasp-and-drag constraint를 반영하였다.

DER 기반 모델은 다음과 같은 기본 물리 현상을 설명하는 데 유용했다.

- 고정단 유지
- drag 이후의 연속적인 형상 변화
- 진동 감쇠와 안정화
- 인접 node 길이 보존

그러나 실제 로프의 재질, table friction, gripper contact, 카메라 검출 오차를 모두 정확하게 모델링하기는 어려웠다. 물리적으로 타당한 전체 거동은 재현했지만, 세부 node 위치에는 반복적인 예측 오차가 남았다.

세부 구현: [`dynamics/der/README.md`](dynamics/der/README.md)

---

### 3.3 물리 모델의 예측 오차를 학습 기반으로 보정할 수 있는가?

> “DER의 물리 구조를 유지하면서, 반복적으로 남는 예측 오차만 학습할 수는 없는가?”

우리는 pure learning model로 전체 dynamics를 다시 학습하는 대신, DER의 출력을 **physical prior**로 유지하였다. Residual GNN은 현재 rope state, DER rollout, grasp node와 target displacement를 입력으로 받고, DER prediction과 실제 결과 사이의 node-wise residual displacement를 예측한다.

초기 residual model은 위치 오차를 크게 줄였지만, 예측된 로프가 늘어나거나 줄어드는 문제가 나타났다. 이에 따라 target rope의 edge length 분포를 보존하는 **target-edge loss**를 추가하였다. 이 과정에서 node position accuracy와 edge consistency 사이의 trade-off를 확인했고, 후속 MPC와 policy 학습에는 edge consistency가 강화된 모델을 사용하였다.

대표 결과:

- DER baseline 평균 RMSE: **7.67 mm**
- Residual GNN 평균 RMSE: **2.41 mm**
- Edge relative error: **12.9% → 3.1%**

세부 구현: [`dynamics/residual_gnn/README.md`](dynamics/residual_gnn/README.md)

---

### 3.4 수많은 grasp-and-drag 후보 중 무엇을 실행할 것인가?

> “예측 모델이 생겼다면, 어떤 action이 목표 형상에 가장 가까워지는가?”

각 node에 대해 이동 방향과 stroke length를 조합하면 많은 action candidate가 생성된다. 모든 candidate를 동일한 비용으로 정밀 평가하면 계산량이 커지므로, 먼저 방향 적합성과 overshoot 가능성을 이용해 후보를 줄이고, shortlist에 대해서만 DER + GNN rollout을 수행하였다.

초기 1-step MPC는 action 직후의 predicted shape error가 가장 작은 후보를 선택하였다. 이 구조만으로도 중간평가에서 **76.90 / 100으로 Task 2 1위**를 기록하여, 모델 기반 candidate selection이 실제 로봇에서도 유효함을 확인하였다.

세부 구현: [`planning/mpc/README.md`](planning/mpc/README.md)

---

### 3.5 지금 한 step만 좋아지는 action이면 충분한가?

> “현재 오차를 가장 많이 줄이는 action이 다음 action까지 고려해도 좋은 선택인가?”

중간평가 로그에서는 목표에 매우 가까워진 뒤 추가 action으로 다시 멀어지는 경우가 관찰되었다. 1-step MPC가 바로 다음 상태만 평가하기 때문에, 다음 상태에서 어떤 조작 가능성이 열리는지와 목표 근처의 안정성을 충분히 고려하지 못한 것이 원인이었다.

이를 해결하기 위해 첫 action 이후의 상태에서 한 번 더 candidate를 평가하는 **2-step MPC**로 확장하였다. 현재 cost보다 future cost에 더 큰 비중을 주어, 단기적인 개선보다 다음 조작까지 고려했을 때 수렴하기 쉬운 action을 우선하였다.

동일 seed 비교에서:

- 성공률: **4/5 → 5/5**
- 평균 step-to-success: **15.50 → 9.33**
- 최종 mean error: **4.408 mm → 3.685 mm**

2-step을 넘어 horizon을 더 늘리면 계산량이 후보 수에 따라 급격히 증가한다. 따라서 2-step MPC는 teacher quality와 계산 비용 사이의 현실적인 절충안이었다.

---

### 3.6 느린 MPC를 실제 실행 속도의 policy로 바꿀 수 있는가?

> “좋은 teacher는 만들었지만, 매 action마다 MPC를 반복해야 하는가?”

MPC는 좋은 action을 생성하지만, candidate마다 DER rollout과 GNN prediction을 반복하므로 실제 로봇에서 계속 사용하기에는 느렸다. 우리는 2-step MPC의 선택 결과를 teacher dataset으로 만들고, 이를 빠르게 근사하는 policy를 학습하였다.

두 종류의 학습 전략을 사용하였다.

- **Behavior Cloning:** high-quality teacher step을 직접 모방하여 안정적인 초기 policy를 학습
- **Candidate-aware Offline RL:** selected action뿐 아니라 같은 state에서 탈락한 candidate의 상대적 품질과 ranking도 critic 학습에 사용

초기 RL에서는 극소수의 candidate outlier가 매우 큰 Q target을 만들면서 critic이 발산하였다. candidate error clipping과 Q-scale 조정을 적용한 뒤 학습이 안정화되었고, 기존 policy보다 목표 접근성과 edge consistency가 개선되었다.

세부 구현: [`policy/candidate_aware_rl/README.md`](policy/candidate_aware_rl/README.md)

---

### 3.7 학습된 action을 그대로 실제 로봇에 보내도 되는가?

> “시뮬레이션에서 좋은 action이 실제 gripper에서도 같은 방향과 크기로 전달되는가?”

실제 로봇에서는 policy가 적절한 grasp node와 target displacement를 선택하더라도, gripper의 회전 방향, 접촉 상태, endpoint의 유연성, release 이후의 복원력 때문에 동일한 형상 변화가 보장되지 않았다. 대표적으로 다음 문제가 확인되었다.

- gripper rotation 명령과 실제 rope tangent 변화 사이의 sign mismatch
- node 18–19 endpoint에서의 불안정한 회전 전달
- 목표 형상에 가까워진 뒤 추가 action으로 score가 감소하는 현상
- release 직후 로프가 이완되거나 튀면서 형상이 무너지는 현상
- rope observation failure와 workspace boundary 문제

이에 따라 policy 출력 뒤에 **geometry-aware runtime safety layer**를 추가하였다.

- **Directed tangent:** node index가 증가하는 방향을 기준으로 현재 로프와 목표 로프의 local orientation 차이를 계산한다.
- **Rotation probe:** 실제 robot에서 회전 명령과 tangent 변화의 부호 관계를 측정하고, 필요한 경우에만 부분적인 sign correction을 적용한다.
- **Geometry risk control:** direction reversal, overshoot, servo angle boundary를 검사하여 위험한 회전량을 감쇠하거나 제한한다.
- **Endpoint-specific correction:** node 18–19에서는 maximum rotation과 correction ratio를 중앙부보다 보수적으로 설정한다.
- **Initial correction:** 초기 step에서는 중앙부 node를 우선 조작하고, 필요하면 initial push로 목표 형상이 만들어질 공간을 확보한다.
- **Score hold:** 남은 시간과 현재 score를 기준으로 불필요한 추가 action을 중단하여 이미 확보한 형상을 보존한다.
- **Failure recovery:** 로프 검출 실패나 workspace violation이 발생하면 reset 또는 재관측을 수행한다.

특히 **pre-release score hold**는 drag가 끝난 직후 gripper를 열기 전에 score를 확인하는 절차이다. Score가 threshold 이상이면 gripper를 닫은 상태로 유지하여 현재 형상을 물리적으로 고정한다. Hold 중에도 score를 계속 확인하며, 점수가 유지되면 마지막 평가 시점까지 release하지 않고, threshold 아래로 하락하면 hold를 해제한 뒤 release 또는 다음 correction을 수행한다. 이를 통해 gripper를 여는 순간 발생하는 로프의 탄성 복원과 형상 이완으로 최종 score가 감소하는 문제를 줄였다.

실제 로그에서 분석 가능한 18개 action 중 14개인 **77.8%**에서 평균 node error가 감소했으며, 전체 평균 오차도 **173.69 mm에서 106.23 mm**로 줄었다. 이는 실제 로봇에서 실행된 action이 전반적으로 목표 형상에 가까워지는 방향으로 작동했음을 보여준다. 다만 이 수치는 policy와 runtime safety가 결합된 전체 실행 결과이며, 두 계층의 기여도를 분리한 ablation 결과는 아니다.

세부 구현: [`runtime/README.md`](runtime/README.md)

---

## 4. Final Closed-Loop Execution

```python
while time_remaining:
    current_rope = observe_rope()
    current_nodes = extract_ordered_nodes(current_rope, num_nodes=20)
    state = map_to_workspace(current_nodes)

    action = policy_or_mpc(state, goal_state)
    safe_action = runtime_geometry_check(action, state, goal_state)

    grasp(safe_action.node)
    align_local_tangent(safe_action.rotation)
    drag_to(safe_action.target)

    score = evaluate_before_release()
    if should_hold(score, time_remaining):
        while time_remaining and hold_condition_is_valid(score):
            hold_gripper_closed()
            score = reevaluate_score()
        if not time_remaining:
            break

    release()
    reobserve_and_continue()
```

핵심은 `predict once and execute open-loop`가 아니라, **observe → select → execute → re-observe**를 반복하는 것이다. 로봇이 실제로 만든 다음 상태가 모델 예측과 다르더라도, 다음 step에서는 관측된 실제 상태에서 다시 계획을 시작한다.

---

## 5. Module Documentation

| 모듈 | 역할 | 세부 문서 |
|---|---|---|
| DER Dynamics | length, bending, damping, fixed-end, drag constraint | [`dynamics/der/`](dynamics/der/README.md) |
| Residual GNN | DER prediction error correction, target-edge loss | [`dynamics/residual_gnn/`](dynamics/residual_gnn/README.md) |
| MPC | candidate generation, shortlist, 1-step / 2-step planning | [`planning/mpc/`](planning/mpc/README.md) |
| Candidate-aware Offline RL | BC warm-start, candidate-aware actor-critic, qsafe stabilization | [`policy/candidate_aware_rl/`](policy/candidate_aware_rl/README.md) |
| Runtime Safety | tangent alignment, rotation probe, endpoint control, pre-release score hold | [`runtime/`](runtime/README.md) |

### Shared Mapping Dependency

Task 1과 Task 2는 카메라 좌표를 실제 robot workspace 명령으로 변환하는 공통 Mapping 모듈을 사용한다. Mapping은 두 task와 병렬인 저장소 상위 모듈로 분리하며, robot별 calibration, LUT, safe probe와 좌표 변환 과정은 별도의 [`mapping/README.md`](../mapping/README.md)에서 설명한다. 본 Task 2 문서에서는 mapping 결과를 입력으로 사용하되, calibration 방법론 자체는 반복해서 설명하지 않는다.

---

## 6. What Worked, What Did Not

| 관찰된 문제 | 처음 시도 | 최종 판단 및 개선 |
|---|---|---|
| 실제 rope dynamics 오차 | DER parameter tuning만으로 해결 | DER를 physical prior로 유지하고 residual GNN으로 보정 |
| 예측 rope 길이 왜곡 | node position loss 중심 학습 | target-edge loss와 goal-edge projection 적용 |
| 목표 근처에서 다시 멀어짐 | 1-step MPC | future cost를 포함한 2-step MPC |
| MPC 실행 속도 | 매 step online MPC | BC와 candidate-aware offline RL로 근사 |
| RL critic 발산 | raw candidate error를 Q target에 반영 | outlier 분석, clipping, Q-scale 조정 |
| 회전 방향 오류 | 명령 sign을 기하학적으로만 추정 | 실제 robot rotation probe로 sign convention 측정 |
| endpoint 형상 붕괴 | 모든 node에 동일한 회전 규칙 | endpoint 전용 limit과 보수적 correction |
| 고득점 이후 점수 하락 | 계속 action 수행 | 남은 시간 기반 score hold와 pre-release check |
| thicker red rope 성능 하락 | 기존 rope 중심 모델과 threshold | observation/dynamics domain gap을 최종 한계로 확인 |

---


## 7. Repository Structure

```text
task2/
├── README.md
├── dynamics/
│   ├── der/
│   │   └── README.md
│   └── residual_gnn/
│       └── README.md
├── planning/
│   └── mpc/
│       └── README.md
├── policy/
│   └── candidate_aware_rl/
│       └── README.md
└── runtime/
    └── README.md
```

## 8. Key Takeaway

K-DAS Task 2의 핵심은 하나의 복잡한 모델을 만드는 것이 아니라, 서로 다른 장점을 가진 계층을 연결한 데 있다.

- DER는 로프의 기본 물리 구조를 제공한다.
- Residual GNN은 실제 환경에서 반복되는 모델 오차를 보정한다.
- 2-step MPC는 미래 조작 가능성을 고려한 teacher action을 만든다.
- BC와 offline RL은 이를 빠르게 실행 가능한 policy로 변환한다.
- Runtime safety는 실제 gripper와 rope의 불확실성을 마지막 단계에서 보정한다.

결과적으로 Task 2 solution은 단순한 learned policy가 아니라, **physics-informed prediction, future-aware planning, learned action selection, geometry-aware execution을 결합한 closed-loop deformable object manipulation system**으로 발전하였다.
