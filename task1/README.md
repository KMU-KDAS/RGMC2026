<img src="../images/t1_overview01.png" alt="Task 1 planar pushing execution examples" width="950"/>

# Task 1 — Model-Based Closed-Loop Planar Pushing

**IEEE ICRA 2026 Cloud Robotics Competition · Team K-DAS · Kookmin University**

Task 1의 목표는 로봇 푸셔만을 사용하여 평면 위의 물체를 목표 위치와 목표 방향으로 이동시키는 것이다. 물체를 잡거나 들어 올릴 수 없기 때문에, 물체 표면의 어느 지점을 어느 방향으로 얼마나 밀어야 하는지를 매 step마다 결정해야 한다.

K-DAS는 물체를 중심 위치와 회전각, contour/keypoint로 표현하고, **minimum-jerk reference path**, **candidate-wise 1-step physics prediction**, **progress-aware scoring**, **Direct–L–U–A\* safe approach planning**, 그리고 **실행 후 재관찰 기반 폐루프 재계획**을 결합한 planar pushing system을 개발하였다.

> 이 문서는 Task 1 전체의 문제 정의, 개발 흐름, 최종 시스템과 성과를 설명하는 상위 README이다. 물리 모델, 후보 생성, 접근 경로, 형상별 정합과 실행 최적화의 상세 내용은 각 하위 모듈 문서에서 분리해 설명한다.

---

## 1. Competition Result

| 항목 | K-DAS 결과 |
|---|---:|
| Task 1 Final Score | **49.69** |
| Task 1 Ranking | **1st place** |
| Evaluation Objects | **6 objects** |
| Surprise / Unseen Shapes | **3 objects** |
| Final Competition | **Overall 1st place** |

<img src="../images/t1_overview02.png" alt="Task 1 final ranking" width="850"/>

최종 평가에서는 Circle, Square, T, T-long과 함께 Plus 및 Organic surprise shape가 사용되었다. 각 물체마다 5회의 run을 수행하고 상위 3개 run의 평균이 반영되었다. K-DAS는 Task 1에서 49.69점을 기록해 1위를 달성했으며, Task 2의 상위권 성과와 결합되어 종합 우승으로 이어졌다.

<img src="../images/t1_overview03.png" alt="Task 1 object-wise robustness" width="850"/>

| Object | Top-3 Average Score |
|---|---:|
| Circle | **60.49** |
| Square | **55.89** |
| T | **52.24** |
| T-long | **57.21** |
| Plus | **34.47** |
| Organic | **37.82** |

T와 T-long은 비대칭 형상으로 인해 회전 정합이 어렵지만, 기준축 보정과 8-point alignment를 적용한 뒤 비교적 높은 성능을 유지했다. 반면 Plus와 Organic은 concave 또는 비정형 surprise shape로, contact sampling과 rigid-template alignment의 한계가 더 크게 나타났다.

---

## 2. System at a Glance

<img src="../images/t1_overview04.png" alt="Task 1 closed-loop planning pipeline" width="1000"/>

```text
Observe object pose and contour
        ↓
Generate a smooth reference pose path
        ↓
Sample contact point, direction, and stroke candidates
        ↓
Predict each candidate with a 1-step rigid pushing model
        ↓
Score expected progress, direction, overshoot, and path cost
        ↓
Plan a collision-free approach path
        ↓
Approach → push → retreat
        ↓
Re-observe the actual pose and update the next plan
```

Task 1은 한 번의 큰 push로 목표를 맞추는 open-loop 방식이 아니다. 한 번의 action을 실행한 뒤 실제 물체 상태를 다시 관찰하고, 관측된 결과에서 새로운 candidate를 생성하는 **closed-loop replanning system**이다. 따라서 물리 모델의 한 step 예측이 완벽하지 않더라도, 실행 오차가 장시간 누적되는 것을 줄일 수 있다.

---

## 3. Why Planar Pushing Is Difficult

Planar pushing은 단순한 2D 위치 이동 문제가 아니다.

- 로봇은 물체를 **잡거나 들어 올릴 수 없고**, 점 접촉 기반 push만 사용할 수 있다.
- 접촉점이 조금만 달라져도 병진과 회전 비율이 크게 바뀐다.
- 질량 중심, 마찰, 관성모멘트와 실제 접촉 위치를 정확히 알기 어렵다.
- 목표에 빠르게 접근하려는 긴 stroke는 overshoot와 workspace 이탈을 만들 수 있다.
- T, Plus, Organic처럼 비대칭·concave 형상은 pose와 keypoint 정합이 어렵다.
- 원격 robot cell마다 camera와 calibration 차이가 존재하며, 서버 지연도 제한 시간에 영향을 준다.

K-DAS는 이 문제를 **관측 → 후보 생성 → 물리 예측 → 평가 → 안전 실행 → 재관측**의 반복 구조로 분해했다.

---

## 4. Development Story

### 4.1 물체 상태를 무엇으로 표현할 것인가?

> “중심 위치만 맞으면 목표 형상에 도달했다고 볼 수 있는가?”

물체의 상태는 중심 위치와 회전각으로 표현한다.

$$
q=[x,\ y,\ \theta]
$$

하지만 실제 성공 여부는 pose 숫자만이 아니라 현재 contour와 target contour가 작업 공간에서 얼마나 겹치는지를 기준으로 판단해야 한다. 사각형과 T자형은 keypoint polygon으로, 원형은 sampled contour polygon으로 표현했다.

<img src="../images/t1_overview05.png" alt="IoU-based success criterion" width="680"/>

$$
\mathrm{IoU}=\frac{\mathrm{Area}(S_{current}\cap S_{target})}{\mathrm{Area}(S_{current}\cup S_{target})}
$$

IoU는 위치 오차와 회전 오차를 실제 형상 겹침으로 함께 반영하기 때문에 Task 1의 주요 성공 지표로 사용되었다.

세부 문서: [`geometry/README.md`](geometry/README.md)

---

### 4.2 최종 목표를 한 번에 향해 밀어야 하는가?

> “큰 push 한 번이 빠르지만, 목표를 지나쳐 버리면 어떻게 할 것인가?”

현재 pose와 target pose 사이에 minimum-jerk reference path를 생성했다.

$$
s(t)=10t^3-15t^4+6t^5
$$

$$
q_{ref}(t)=q_{current}+s(t)(q_{target}-q_{current})
$$

이 경로는 시작과 끝에서 변화가 급격하지 않으며, 최종 목표 대신 현재 단계에서 따라가야 할 intermediate target을 제공한다.

<img src="../images/t1_overview06.png" alt="Candidate-wise reference target selection" width="820"/>

중요한 개선은 모든 후보를 동일한 intermediate target과 비교하지 않았다는 점이다. 각 candidate의 predicted next pose가 reference path에서 도달할 수 있는 위치를 찾고, 그 후보에 맞는 평가 지점을 따로 선택했다. 짧은 stroke와 긴 stroke를 같은 기준으로 평가하면서 생기던 편향을 줄인 것이다.

세부 문서: [`planning/README.md`](planning/README.md)

---

### 4.3 어떤 지점을 어느 방향으로 얼마나 밀 것인가?

물체의 면 또는 contour에서 contact point를 sampling하고, 표면 법선 방향과 stroke length를 조합해 candidate action을 생성했다.

<img src="../images/t1_overview07.png" alt="Push candidate generation" width="760"/>

각 candidate는 다음 요소로 구성된다.

- contact point on object boundary
- pusher center start point
- inward push direction
- stroke length
- approach and retreat positions

Pusher는 실제 반지름을 갖기 때문에, 로봇 중심이 물체 표면에 놓이지 않도록 contact point에서 바깥쪽으로 offset한 시작점을 사용했다. 목표가 멀 때는 긴 stroke를, 목표에 가까울 때는 짧은 correction stroke를 선호하도록 거리에 따른 길이 설정을 적용했다.

세부 문서: [`planning/README.md`](planning/README.md)

---

### 4.4 실행 전에 결과를 예측할 수 있는가?

> “이 contact point를 밀었을 때 물체는 얼마나 이동하고 회전하는가?”

각 candidate는 실제 실행 전에 rigid pushing model로 한 step 예측된다.

<img src="../images/t1_overview08.png" alt="One-step rigid pushing model" width="850"/>

예측 모델은 다음 관계를 단순화해 사용했다.

- applied force and translational acceleration
- contact lever arm and torque
- object mass and moment of inertia
- friction and COM belief
- push duration and stroke distance

실제 로봇에서는 push 전후 pose 차이를 기록하여 observed translation과 rotation을 계산하고, 다음 planning에서 COM belief와 예측–실측 차이를 반영할 수 있도록 loop를 확장했다.

세부 문서: [`dynamics/README.md`](dynamics/README.md)

---

### 4.5 어떤 candidate가 실제로 더 좋은가?

단순히 target과의 최종 거리만 비교하면 과도한 stroke나 잘못된 회전 방향이 선택될 수 있다. Candidate score는 다음 요소를 함께 고려했다.

<img src="../images/t1_overview09.png" alt="Candidate scoring concept" width="850"/>

- reference path progress
- translation and orientation error
- motion direction consistency
- overshoot risk
- workspace boundary margin
- approach path cost

개념적으로는 다음과 같이 정리할 수 있다.

$$
J = w_e E_{pose} - w_p P_{progress} + w_o R_{overshoot} + w_c C_{path}
$$

가장 낮은 cost를 갖는 candidate라도 로봇이 안전하게 접근할 수 없다면 실행 후보에서 제외된다.

---

### 4.6 좋은 push라도 시작점에 안전하게 접근할 수 있는가?

> “Candidate는 좋지만 로봇이 물체와 충돌하지 않고 접촉 시작점까지 갈 수 있는가?”

접근 경로는 계산량과 실행 시간을 고려해 단계적으로 탐색했다.

<img src="../images/t1_overview10.png" alt="Direct L U and A-star approach planning" width="900"/>

```text
Direct path
    ↓ failure
L-shaped path
    ↓ failure
U-shaped path
    ↓ failure
Short A* path
```

처음에는 Direct → L-shape → A\*를 사용했지만, L-shape가 실패하면 곧바로 긴 A\* 경로가 생성되어 실행 시간이 크게 증가했다. 이를 완화하기 위해 U-shape를 추가하고, A\* waypoint 제한도 `5 → 7 → 9 → 12` 순서로 점진적으로 완화했다. 짧은 경로를 먼저 시도하면서도, 필요한 경우 마지막 feasible path를 놓치지 않는 구조다.

세부 문서: [`planning/README.md`](planning/README.md)

---

### 4.7 모델 예측과 실제 실행이 다르면 어떻게 할 것인가?

<img src="../images/t1_overview11.png" alt="Simulation to real-robot development flow" width="900"/>

초기에는 물리 모델 시뮬레이션으로 candidate generation, scoring과 반복 planning의 논리를 검증했다. 이후 같은 planner를 실제 robot observation과 연결하고, 매 push 이후 다음 절차를 반복했다.

1. 물체와 gripper를 다시 검출한다.
2. Pixel coordinates를 workspace coordinates로 변환한다.
3. Predicted pose와 observed pose를 비교한다.
4. 실제 motion과 rotation response를 기록한다.
5. Retreat 후 다음 plan을 새로 계산한다.

이 구조는 physics simulation을 그대로 믿는 것이 아니라, **실제 관측값을 다음 planning의 새로운 초기 상태로 사용**한다.

---

### 4.8 비대칭 형상과 대칭 형상을 같은 방식으로 정렬할 수 있는가?

<img src="../images/t1_overview12.png" alt="T-shape basis and rigid alignment correction" width="950"/>

#### T-shape

T자형에서는 detection의 stem 방향과 planner canonical shape의 기준축이 약 90° 어긋나는 문제가 발생했다. Current shape와 target shape에 동일한 basis correction을 적용하고, 단일 중심점 대신 **8개 꼭짓점 전체를 사용하는 rigid alignment**로 target pose를 계산했다.

#### Square

사각형은 90° rotational symmetry를 갖기 때문에, 같은 자세라도 contour detector가 어느 꼭짓점을 첫 번째로 반환하는지에 따라 vertex index가 달라질 수 있다. 따라서 네 가지 cyclic permutation을 모두 비교하고 최소 shape error를 사용했다.

이 보정은 실제로는 정렬된 사각형을 불필요하게 다시 회전시키는 문제를 줄였다.

세부 문서: [`geometry/README.md`](geometry/README.md)

---

### 4.9 제한 시간 안에 더 많은 유효한 push를 실행할 수 있는가?

중간평가 로그를 분석한 결과, 가장 큰 시간 병목은 planner 계산 자체보다 서버 응답, waypoint별 sleep과 긴 approach path 실행이었다.

적용한 개선은 다음과 같다.

- waypoint별 고정 sleep 감소
- Direct/L/U path 우선 사용
- A\* waypoint 수 제한과 path simplification
- 목표 거리 기반 stroke 확대
- push 이후 retreat distance 최적화
- 실패 경로의 빠른 탈락과 다음 후보 전환

긴 stroke는 빠른 접근에 유리했지만 workspace 이탈과 overshoot를 증가시켰다. 따라서 목표까지의 거리뿐 아니라 경계 margin을 함께 고려해야 했다.

---

## 5. Final Closed-Loop Execution

```python
while time_remaining:
    observation = capture_bottom_view()
    current_shape = detect_object(observation)
    robot_xy = detect_or_read_pusher_state(observation)

    current_pose = estimate_pose_and_keypoints(current_shape)
    target_on_path = build_reference_path(current_pose, target_pose)

    candidates = generate_push_candidates(current_shape, target_on_path)
    predicted = [predict_one_step(current_pose, action) for action in candidates]
    selected = score_and_select(candidates, predicted, target_on_path)

    approach = plan_safe_approach(
        robot_xy,
        selected.start,
        obstacle=current_shape,
        methods=["direct", "L", "U", "A_star"],
    )

    execute_approach_push_retreat(approach, selected)
    observed_next = reobserve_object()
    update_motion_and_com_belief(current_pose, selected, observed_next)

    if evaluator_iou(observed_next, target_shape) >= success_threshold:
        break
```

핵심은 model prediction으로 action을 선택하되, 실행 후에는 반드시 실제 observation으로 돌아오는 것이다.

---

## 6. Module Documentation

| 모듈 | 역할 | 세부 문서 |
|---|---|---|
| Geometry & Shape Alignment | pose, polygon, IoU, symmetry and target alignment | [`geometry/`](geometry/README.md) |
| Rigid Pushing Dynamics | force/torque-based 1-step prediction and COM belief | [`dynamics/`](dynamics/README.md) |
| Planning & Candidate Selection | reference path, contact sampling, scoring, Direct/L/U/A* approach planning | [`planning/`](planning/README.md) |
| Runtime Execution | approach, push, retreat, latency and recovery handling | [`runtime/`](runtime/README.md) |

### Shared Mapping Dependency

Task 1과 Task 2는 camera pixel coordinates를 실제 CloudGripper workspace coordinates로 변환하는 공통 Mapping 모듈을 사용한다. Robot별 calibration, LUT, safe probe와 좌표 변환 방법은 Task 1 내부에 중복해서 설명하지 않고, repository 상위의 [`mapping/README.md`](../mapping/README.md)에서 별도로 다룬다.

---

## 7. Development Validation

최종 대회 이전의 반복 실험에서는 다음 결과를 확인했다.

| Validation | Result |
|---|---:|
| Square tests reaching final IoU ≥ 0.8 | **11 / 11** |
| Best square final IoU | **0.90** |
| Best square score | **132.40** |
| Square average steps | **4.0** |
| Square average time per step | **24.41 s** |
| Circle tests reaching IoU ≥ 0.8 | **5 / 8** |
| Best circle Max IoU | **0.96** |
| Fastest successful circle run | **49.86 s** |

Square의 cyclic vertex matching과 approach planner 개선 이후 11회 테스트 모두 IoU 0.8 이상을 기록했다. Circle에서는 긴 stroke로 빠르게 접근할 수 있었지만, 일부 run에서 workspace 이탈과 overshoot가 발생해 5/8 성공률을 기록했다.

<img src="../images/t1_overview13.png" alt="Task 1 top and low score run comparison" width="900"/>

---

## 8. Failure Cases and Lessons

| Failure mode | Cause | Mitigation / lesson |
|---|---|---|
| Overshoot and workspace exit | stroke too long near the boundary | distance-aware stroke + boundary margin |
| Unnecessary rotation of square | inconsistent vertex start index | cyclic vertex permutation matching |
| T-shape orientation mismatch | detection and canonical axes differed by about 90° | shared basis correction + 8-point alignment |
| Excessive execution time | long A\* path and waypoint sleep | Direct/L/U priority + progressive A\* limit |
| Candidate exists but no approach path | safety margin rejected contact-near poses | distinguish object collision from necessary contact approach |
| Prediction–execution mismatch | friction, COM, contact and robot errors | re-observation, COM belief update and replanning |
| Low Plus / Organic performance | concavity and ambiguous keypoints | contour-level matching and richer contact dynamics are needed |

---

## 9. Task 1 Directory

```text
task1/
├── README.md
├── geometry/
│   └── README.md
├── dynamics/
│   └── README.md
├── planning/
│   └── README.md
└── runtime/
    └── README.md
```

Mapping은 Task 1 폴더 안에 포함하지 않고, Task 1·Task 2가 함께 사용하는 repository-level module로 둔다.

---

## 10. Takeaway

K-DAS Task 1은 단순히 목표 방향으로 물체를 반복해서 미는 rule-based system이 아니다. 물체의 pose와 contour를 관측하고, smooth reference path 위에서 push candidate를 생성한 뒤, 각 후보를 물리 모델로 예측하고 안전하게 실행하는 **model-based closed-loop planar pushing system**이다.

최종적으로 이 구조는 seen object뿐 아니라 surprise shape가 포함된 평가에서 Task 1 **49.69점, 1위**를 기록했다. 동시에 Plus와 Organic 결과는 rigid pose와 sparse keypoint만으로는 복잡한 concave shape를 완전히 다루기 어렵다는 후속 연구 과제도 보여준다.
