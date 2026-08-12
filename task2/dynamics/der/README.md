<img src="../../../images/t2_der01.png" alt="DER module pipeline" width="900"/>

# DER-Inspired Planar Rope Dynamics

**Task 2 · Linear Deformable Object Shape Control · Team K-DAS**

이 모듈은 현재 로프 형상과 grasp-and-drag action이 주어졌을 때, action 이후의 로프 형상을 예측하는 **물리 기반 baseline dynamics model**이다. 로프를 20개의 ordered node와 19개의 edge로 표현하고, 길이 보존, 굽힘 저항, damping, 고정단, grasp node의 drag constraint를 이용해 다음 상태를 계산한다.

> 본 구현은 Bergou et al.의 3차원 Discrete Elastic Rods 전체를 그대로 재현한 것이 아니다. Task 2의 top-view manipulation에 필요한 **centerline, bending, inextensibility** 개념을 2차원 평면 문제에 맞게 단순화한 **DER-inspired planar model**이다.

---

## 1. Why a Physics Model?

Task 2에서 planner는 단순히 현재 로프와 목표 로프의 차이를 계산하는 것만으로는 충분하지 않다. 특정 node를 잡고 이동했을 때 나머지 node들이 어떻게 따라 움직이는지를 예측해야 여러 action candidate를 비교할 수 있다.

```text
Current state X_t + Action a_t
                ↓
        Dynamics prediction
                ↓
       Predicted state X̂_t+1
                ↓
   Candidate cost and action selection
```

초기 목표는 다음 질문에 답하는 것이었다.

> **“어떤 node를 어느 위치로 이동시키면 로프 전체 형상은 어떻게 변하는가?”**

순수한 규칙 기반 변위 전파는 현재 곡률과 grasp 위치에 따른 차이를 설명하기 어렵고, pure learning model은 충분한 데이터가 없을 때 물리적으로 비정상적인 형상을 예측할 수 있다. 따라서 먼저 로프의 구조적 성질을 반영하는 물리 모델을 만들고, 이후 남는 오차만 Residual GNN으로 보정하는 방향을 선택하였다.

---

## 2. Role in the Full Task 2 System

DER는 최종 controller 자체가 아니라, 후속 모듈들이 사용하는 **physical prior**이다.

1. 현재 로프를 20개 ordered node로 변환한다.
2. action candidate마다 DER rollout을 수행한다.
3. Residual GNN이 DER prediction의 systematic error를 보정한다.
4. MPC 또는 learned policy가 보정된 다음 상태를 이용해 action을 비교한다.
5. 실제 action 후에는 로프를 다시 관측하고 다음 step을 계획한다.

관련 문서:

- 상위 Task 2 개요: [`../../README.md`](../../README.md)
- Residual correction: [`../residual_gnn/README.md`](../residual_gnn/README.md)
- MPC planning: [`../../planning/mpc/README.md`](../../planning/mpc/README.md)

---

## 3. Modeling Scope and Assumptions

| 항목 | 적용 방식 |
|---|---|
| 공간 | Top-view 기반 2D plane |
| 로프 표현 | 20 ordered nodes, 19 edges |
| 고정 조건 | 한쪽 끝 node를 고정 |
| 내부 물리 | Edge length preservation, bending resistance, damping |
| 조작 입력 | 선택 node를 target position으로 이동시키는 grasp-and-drag action |
| 접촉 모델 | Table friction과 gripper contact를 단순화된 계수와 constraint로 처리 |
| 제외한 항목 | 3D torsion, self-collision의 정밀 접촉, 재질별 비선형 마찰, gripper jaw deformation |

이 단순화의 목적은 실제 로프의 모든 물리 현상을 완벽히 재현하는 것이 아니라, action candidate의 상대적인 품질을 비교할 수 있는 **빠르고 구조화된 rollout model**을 확보하는 것이다.

---

## 4. State and Action Representation

### 4.1 Rope State

시간 `t`에서의 로프 상태는 node 좌표의 순서 있는 배열로 표현한다.

$$
X_t = [x_0, x_1, \ldots, x_{N-1}], \qquad x_i \in \mathbb{R}^2, \quad N=20
$$

Node index는 고정단에서 자유단으로 증가한다. 이 순서는 edge, tangent, local curvature, grasp node의 영향을 일관되게 계산하기 위해 유지되어야 한다.

### 4.2 Manipulation Action

최소 action 표현은 다음과 같다.

$$
a_t = (g, p_{\mathrm{target}})
$$

- `g`: grasp할 node index
- `p_target`: grasp node를 이동시킬 workspace target

실제 rollout에서는 drag distance, action duration 또는 robot speed configuration을 이용해 grasp node의 이동 경로를 시간 구간으로 나누어 반영한다.

### 4.3 Transition Dataset

실제 로봇 데이터는 다음 transition으로 저장된다.

$$
X_t + a_t \rightarrow X_{t+1}
$$

이 transition은 DER parameter fitting, DER baseline 생성, Residual GNN 학습과 validation에 공통으로 사용된다.

---

## 5. Planar DER Formulation

### 5.1 Edge and Unit Tangent

인접 node를 연결하는 edge와 단위 접선은 다음과 같다.

$$
e_i = x_{i+1} - x_i
$$

$$
t_i = \frac{e_i}{\lVert e_i \rVert}
$$

각 edge의 초기 길이 `l_i^0`는 로프의 rest length constraint로 사용된다.

### 5.2 Bending Angle

인접 edge 사이의 회전각을 통해 국소 굽힘을 표현한다.

$$
\phi_i = \cos^{-1}(t_{i-1} \cdot t_i)
$$

직선에 가까울수록 `φ_i`가 작고, node 주변이 급격하게 꺾일수록 값이 커진다.

### 5.3 Bending Energy

평면 모델의 굽힘 에너지는 다음과 같이 구성하였다.

$$
E_{\mathrm{bend}} = \frac{1}{2}\sum_i k_b\phi_i^2
$$

`k_b`는 bending stiffness이다. 값이 클수록 로프가 직선 형상을 강하게 유지하며, 값이 작을수록 동일한 입력에서 더 쉽게 굽어진다.

### 5.4 Length Preservation

로프가 action 과정에서 비정상적으로 늘어나거나 줄어드는 것을 막기 위해 각 edge에 길이 제약을 둔다.

$$
C_i = \lVert e_i \rVert - l_i^0 = 0
$$

본 구현에서는 내부력 계산과 별도로 projection 단계에서 edge length를 반복 보정한다. 이 방식은 explicit stiffness force만 크게 설정하는 것보다 수치적으로 안정적이며, action 이후에도 전체 로프 길이를 비교적 일관되게 유지한다.

### 5.5 Damping and Node Motion

각 node의 운동은 굽힘력, 감쇠력, 외력 또는 조작 constraint의 합으로 계산한다.

$$
m_i\ddot{x}_i = f_i^{\mathrm{bend}} + f_i^{\mathrm{damp}} + f_i^{\mathrm{external}}
$$

감쇠항은 입력 이후 진동이 계속 증가하지 않고 안정화되도록 한다.

$$
f_i^{\mathrm{damp}} = -d\dot{x}_i
$$

`d`가 지나치게 작으면 진동이 오래 남고, 지나치게 크면 실제 로프보다 변형 전달이 둔해진다.

---

## 6. Boundary and Manipulation Constraints

### 6.1 Fixed End

고정단 node는 전체 rollout 동안 초기 위치를 유지한다.

$$
x_0(t)=x_0(0), \qquad \dot{x}_0(t)=0
$$

이 조건이 깨지면 자유단의 변형을 계산하더라도 전체 로프가 workspace에서 이동해 버리므로, 매 simulation step에서 강제로 적용한다.

### 6.2 Grasp-and-Drag Constraint

선택된 grasp node는 robot action과 동일한 방향으로 target까지 이동하도록 유도한다. 이를 순간적인 impulse 하나로 처리하지 않고, drag trajectory를 시간 구간으로 나누어 적용한다. 그 결과 action 도중 인접 node가 연속적으로 따라오도록 한다.

### 6.3 Edge Projection

위치가 갱신된 뒤 각 edge가 rest length에 가까워지도록 node position을 보정한다. Fixed node와 grasp node의 constraint를 우선 유지하면서 나머지 node에 correction을 분배한다.

---

## 7. Numerical Rollout

DER rollout의 한 simulation step은 다음 순서로 구성된다.

```python
for step in simulation_steps:
    edges = compute_edges(nodes)
    tangents = normalize(edges)

    bending_force = compute_bending_force(nodes, tangents, kb)
    damping_force = -damping * velocity

    apply_grasp_drag_constraint(grasp_node, drag_trajectory[step])
    integrate_node_motion(bending_force + damping_force)

    project_edge_lengths(rest_lengths)
    enforce_fixed_endpoint(node=0)

return predicted_nodes
```

핵심 순서는 다음과 같다.

1. Edge와 tangent 계산
2. Bending force 계산
3. Damping 적용
4. Grasp-and-drag constraint 적용
5. Node position 갱신
6. Edge length projection
7. Fixed-end constraint 재적용

이 rollout은 모든 candidate에 반복 적용되므로, 물리적 정밀도뿐 아니라 계산 비용과 수치 안정성도 중요하다.

---

## 8. Parameter Identification

<img src="../../../images/t2_der02.png" alt="DER parameter fitting" width="900"/>

DER 구조를 구현한 뒤에도 실제 로프의 `k_b`와 damping은 알려져 있지 않았다. 같은 grasp-and-drag 입력에서도 simulation과 실제 로프의 굽힘 정도와 복원 양상이 달랐기 때문에, 실제 transition data를 이용해 두 계수를 추정하였다.

### 8.1 Initial Inverse Estimation

초기에는 관측된 변형으로부터 계수를 직접 역산하려고 했다.

$$
k_{b,\mathrm{proxy}} = \frac{U}{K_{\mathrm{local}}+\epsilon}
$$

$$
\gamma_{\mathrm{proxy}} = \mathrm{clip}\left(\frac{\bar{d}_{\mathrm{visible}}}{U+\epsilon},0,1\right)
$$

- `U`: action displacement magnitude
- `K_local`: grasp node 주변 curvature change
- `d_visible`: visible node의 평균 displacement

그러나 실제 simulator에는 projection, threshold, clamp와 같은 비연속 연산이 포함되어 있었다. 일부 node만 관측되는 데이터 특성까지 결합되면서, 안정적인 inverse function을 구성하기 어려웠다.

### 8.2 Forward Numerical Optimization

역산 대신, parameter를 바꾸어 DER를 반복 실행하고 실제 다음 상태와의 오차를 직접 최소화하는 정방향 최적화로 전환하였다.

Parameter vector:

$$
\theta=[k_b,d]
$$

Batch objective:

$$
J(k_b,d)=\frac{1}{N}\sum_{i=1}^{N}\sqrt{\frac{1}{M}\sum_{j=1}^{M}\lVert x^{\mathrm{sim}}_{i,j}(k_b,d)-x^{\mathrm{gt}}_{i,j}\rVert_2^2}
$$

- `N`: batch transition 수
- `M`: node 수
- `x_sim`: DER rollout 결과
- `x_gt`: 실제 관측 결과

최적화는 두 단계로 수행하였다.

1. **surrogateopt**: 미분이 어렵고 평가 비용이 큰 parameter space를 전역적으로 탐색
2. **patternsearch**: 전역 탐색 결과 근방에서 gradient 없이 local refinement

단일 sample fitting은 특정 trajectory에 과적합되는 문제가 있어, 20개 sample pilot test 이후 약 200개의 유효 transition 전체로 확장하였다. 전체 최적화에서는 `parpool`과 `parfor`를 사용해 반복 rollout을 병렬화하였다.

### 8.3 Calibrated Parameters

| Parameter | Fitted value |
|---|---:|
| Bending stiffness `k_b` | `3.225057e-05` |
| Damping `d` | `0.7077` |
| Batch normalized RMSE | `0.0977` |
| Approximate physical-space error | `14.65 mm` |

이 parameter는 특정 action 하나에 가장 잘 맞는 값이 아니라, 전체 transition batch의 평균 오차를 줄이도록 선택한 **surrogate physical parameter**이다. 따라서 재질의 정확한 물성값이라기보다, 현재 2D simulator와 데이터 표현에서 가장 일관된 rollout을 제공하는 계수로 해석해야 한다.

---

## 9. Validation

<img src="../../../images/t2_der03.png" alt="DER validation summary" width="850"/>

### 9.1 Physics Sanity Check

초기 직선 rope에 외력을 가한 simulation에서 다음 항목을 검증하였다.

| 검증 항목 | 결과 | 해석 |
|---|---:|---|
| Fixed-end error | `0.0` | 고정단이 전체 구간에서 기준 위치 유지 |
| Kinetic energy | 입력 후 peak, 이후 감소 | 진동이 증폭되지 않고 damping에 의해 안정화 |
| Maximum stretch ratio | `1.85%` | 강한 입력 순간의 일시적 신장 |
| Mean stretch ratio | `0.0112%` | 전체적으로 edge length가 잘 유지됨 |
| Maximum curvature | `65.61` | 약 `t = 2.89 s`에서 일시적인 큰 굽힘 발생 후 감소 |

이 검증은 모델이 실제 데이터를 정확히 맞추는지를 평가한 것이 아니라, 고정단·감쇠·길이 보존과 같은 기본 물리 조건이 수치적으로 무너지지 않는지를 확인한 것이다.

### 9.2 Data-Fitting Result

전체 transition dataset 기반 parameter fitting에서는 실제 결과와 약 **14.65 mm**의 평균 오차를 기록하였다. 전체 곡률과 형상 변화는 대체로 유사했지만, grasp 위치에서 멀리 떨어진 자유단 부근에서는 변형 전달이 과대 또는 과소 예측되는 패턴이 남았다.

### 9.3 Later Baseline Evaluation

후속 Residual GNN 실험의 별도 validation subset 10개에서는 DER baseline이 다음 결과를 보였다.

| Metric | DER baseline |
|---|---:|
| Mean RMSE | `7.67 mm` |
| Mean MAE | `6.24 mm` |

`14.65 mm`와 `7.67 mm`는 서로 다른 dataset과 evaluation protocol에서 계산된 값이므로 직접적인 성능 개선 수치로 비교해서는 안 된다. 전자는 전체 parameter fitting 단계의 batch 평균이며, 후자는 후속 GNN 비교를 위해 선정한 별도 validation sample의 baseline 결과이다.

---

## 10. What the DER Model Explained Well

- 고정단이 유지되는 상태에서 자유단과 중간 node가 연속적으로 변형되는 현상
- Grasp node의 이동이 인접 edge를 따라 주변 node에 전달되는 기본 구조
- Bending stiffness에 따른 로프의 굽힘 정도 변화
- Damping에 따른 진동 감소와 안정화
- Edge projection을 통한 전체 길이와 local edge consistency 유지
- MPC candidate 간 상대적인 action quality를 비교할 수 있는 물리적 baseline

DER의 가장 중요한 역할은 실제 결과를 완벽히 맞추는 것이 아니라, **아무 구조 없이 node displacement를 추정하는 것보다 물리적으로 일관된 출발점**을 제공한 것이다.

---

## 11. Limitations

### 11.1 Parameter Ambiguity

실제 관측 결과에는 rope material뿐 아니라 table friction, gripper contact, drag speed, node detection noise가 함께 반영된다. 따라서 `k_b`와 damping만으로 모든 오차를 설명할 수 없다.

### 11.2 Endpoint Error

Grasp node에서 멀리 떨어진 자유단 node에서는 장력 전달과 누적 변형이 실제와 다르게 나타났다. 이는 단순한 parameter tuning보다 모델 구조의 한계에 가깝다.

### 11.3 Simplified Contact

Table과 rope 사이의 stick-slip friction, gripper jaw와 rope의 접촉면, release 이후의 탄성 복원을 정밀하게 모델링하지 않았다.

### 11.4 2D Assumption

Top-view shape control에 집중했기 때문에 out-of-plane motion과 torsion을 제외하였다. Rope가 들리거나 겹치는 상황에서는 planar model의 정확도가 낮아질 수 있다.

### 11.5 Perception and Mapping Error

DER는 20개 node 좌표를 ground truth로 가정하지만, 실제 입력에는 segmentation, node ordering, pixel-to-workspace mapping error가 포함된다. 따라서 최종 prediction error는 dynamics error만을 의미하지 않는다.

---

## 12. Why Residual GNN Was Added

Parameter optimization 이후에도 반복적인 residual error가 남았다. 특히 세부 node 위치와 endpoint propagation은 물리 계수 두 개만으로 안정적으로 맞추기 어려웠다.

이에 따라 DER를 폐기하지 않고 다음 hybrid structure를 사용하였다.

$$
\hat{X}_{t+1}^{\mathrm{final}} = \hat{X}_{t+1}^{\mathrm{DER}} + \Delta X_{t+1}^{\mathrm{GNN}}
$$

- DER: length, bending, fixed-end와 같은 physical structure 제공
- Residual GNN: 실제 데이터에서 반복되는 node-wise prediction error 보정

별도 validation에서 DER baseline 평균 RMSE `7.67 mm`는 Residual GNN 적용 후 `2.41 mm`까지 감소하였다. 이 결과는 물리 모델이 불필요했다는 의미가 아니라, **DER가 제공하는 구조 위에서 학습 모델이 수정해야 할 범위를 residual로 제한했을 때 더 높은 정확도를 얻을 수 있었다**는 의미이다.

---

## 13. Repository Guide

권장 모듈 구조는 다음과 같다. 실제 공개 시에는 현재 코드 파일명에 맞추어 조정한다.

```text
task2/dynamics/der/
├── README.md
├── src/              # planar DER rollout and constraints
├── configs/          # node count, kb, damping, simulation settings
├── scripts/          # parameter fitting and batch evaluation
├── notebooks/        # visualization and sanity checks
├── results/          # fitting logs and validation tables
```

공개 시 다음 정보를 함께 제공하는 것이 필요하다.

- 좌표 단위와 normalization 범위
- Node ordering과 fixed endpoint 정의
- Rest edge length 생성 방식
- Simulation step 수와 action duration 처리
- Fitted `k_b`, damping과 적용 대상 rope
- Parameter fitting dataset schema
- DER output을 Residual GNN과 MPC에 전달하는 interface

---


> 모든 시각 자료는 repository 최상위 `images/` 폴더에서 공통 관리한다.

## 14. Minimal Interface

```python
predicted_nodes = der_rollout(
    current_nodes=current_nodes,       # shape: [20, 2]
    grasp_node=action.node_index,
    target_position=action.target_xy,
    bending_stiffness=3.225057e-05,
    damping=0.7077,
)
```

Expected output:

```text
predicted_nodes: [20, 2]
```

후속 모듈에서는 이 결과를 그대로 최종 prediction으로 사용하지 않고, Residual GNN correction과 edge-consistency 처리를 적용한 뒤 action cost를 계산한다.

---

## 15. Takeaway

> **The DER module is a fast, physically structured baseline for predicting how a fixed-end rope responds to grasp-and-drag actions.**

K-DAS의 DER 모듈은 로프를 20-node chain으로 이산화하고, bending, damping, inextensibility, fixed-end와 manipulation constraint를 결합하여 기본적인 변형 거동을 재현하였다. 실제 데이터 기반 forward optimization으로 parameter를 보정했지만, endpoint propagation과 contact uncertainty에는 반복적인 오차가 남았다. 이 한계는 이후 DER prediction을 physical prior로 유지하면서 Residual GNN으로 오차를 보정하는 hybrid dynamics architecture로 이어졌다.

---

## References

1. M. Bergou, M. Wardetzky, S. Robinson, B. Audoly, and E. Grinspun, “Discrete Elastic Rods,” *ACM Transactions on Graphics*, vol. 27, no. 3, 2008. [Official publication page](https://research.adobe.com/publication/discrete-elastic-rods/)
2. Task 2 project documentation: [`../../README.md`](../../README.md)
