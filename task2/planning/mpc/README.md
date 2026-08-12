<img src="../../../images/t2_mpc01.png" alt="2-step MPC teacher generation pipeline" width="950"/>

# MPC Teacher for Rope Manipulation

**Task 2 · Linear Deformable Object Shape Control · Team K-DAS**

이 모듈은 DER + Residual GNN hybrid dynamics model을 이용하여 **teacher action**을 생성하는 MPC 계층을 설명한다. 현재 로프에서 가능한 grasp·direction·stroke 후보를 생성하고, 각 후보 이후의 로프 형상을 예측하여 목표 형상에 더 안정적으로 접근하는 action을 선택한다.

> **핵심 질문**  
> 현재 step에서만 좋아 보이는 action이 아니라, 다음 step까지 고려했을 때 더 안정적인 action을 teacher로 만들 수 있는가?

---

## 1. Problem Context

<img src="../../../images/t2_mpc02.png" alt="CloudGripper rope manipulation environment with current and target rope overlay" width="720"/>

<p align="center"><i>CloudGripper 환경에서 관측된 현재 rope와 target rope overlay. MPC는 이 두 형상 사이의 차이를 줄이는 action을 선택한다.</i></p>

Task 2에서는 한쪽 끝이 고정된 로프를 20개의 ordered node로 표현하였다. Action 하나가 선택된 node 주변뿐 아니라 로프 전체 형상에 영향을 주기 때문에, 단순히 현재 오차가 큰 node를 당기는 규칙만으로는 안정적인 제어가 어려웠다.

MPC의 역할은 다음과 같다.

- 가능한 action들을 체계적으로 생성한다.
- DER + Residual GNN으로 action 이후의 rope state를 예측한다.
- 즉시 오차와 future cost를 비교한다.
- 가장 좋은 first action을 teacher transition으로 기록한다.

---

## 2. Why MPC?

DER와 Residual GNN을 통해 다음 상태를 예측할 수 있게 되었지만, 예측 모델만으로는 실제 action을 선택할 수 없다. 특히 rope manipulation에서는 다음 문제가 있었다.

- 현재 step에서 error가 줄어도 다음 step의 조작 가능성이 나빠질 수 있음
- grasp node, direction, stroke length의 조합을 사람이 직접 규칙으로 정하기 어려움
- BC/RL을 학습시키기 위한 일관된 teacher action이 필요함

따라서 hybrid dynamics model을 **candidate evaluator**로 사용하고, MPC가 좋은 action을 선별하도록 구성하였다.

---

## 3. Action Candidate Generation

<img src="../../../images/t2_mpc03.png" alt="Action candidate cloud generated around rope nodes" width="900"/>

<p align="center"><i>각 rope node에서 여러 direction과 stroke length를 조합해 생성한 action candidate cloud.</i></p>

각 action은 다음 세 요소로 표현된다.

$$
a=(i_{\mathrm{grasp}},\theta_{\mathrm{bin}},\ell_{\mathrm{bin}})
$$

- **grasp node index**: 어느 node를 잡을 것인가
- **direction bin**: 어느 방향으로 drag할 것인가
- **stroke length bin**: 얼마나 이동할 것인가

모든 candidate를 동일한 깊이로 탐색하면 계산량이 지나치게 커지므로, 먼저 1-step evaluation을 수행하고 상위 후보만 shortlist로 남긴 뒤 2-step expansion을 적용하였다.

---

## 4. Transition Model: DER + Residual GNN

후보 action 이후의 rope state는 다음 hybrid model로 예측한다.

$$
\hat X_{t+1}=f_{\mathrm{DER+GNN}}(X_t,a_t)
$$

- DER: fixed-end, bending, damping과 기본적인 deformation propagation 제공
- Residual GNN: DER와 실제 로봇 사이의 systematic prediction error 보정

2-step planning에서는 첫 번째 예측 상태에서 다시 action을 적용한다.

$$
\hat X_{t+2}=f_{\mathrm{DER+GNN}}(\hat X_{t+1},a_{t+1})
$$

따라서 MPC 성능은 cost function뿐 아니라 transition model의 정확도와 edge consistency에 영향을 받는다.

---

## 5. Cost and Edge Projection

MPC는 예측된 rope와 target rope의 차이를 cost로 계산한다. 핵심 항은 다음과 같다.

$$
J=w_{\mathrm{pos}}E_{\mathrm{pos}}+w_{\mathrm{shape}}E_{\mathrm{shape}}+w_{\mathrm{edge}}E_{\mathrm{edge}}
$$

- node-wise position error
- 전체 shape와 local tangent 차이
- edge length와 total length consistency

Residual GNN의 예측에서 edge drift가 발생할 수 있으므로, MPC 내부에는 **goal edge length projection**을 추가하였다.

<img src="../../../images/t2_mpc04.png" alt="Projection alpha trade-off" width="780"/>

<p align="center"><i>Projection을 강하게 할수록 edge error는 감소하지만, 지나친 보정은 shape tracking을 악화시켰다.</i></p>

| Projection alpha | Success rate | Final mean error | Final edge error |
|---:|---:|---:|---:|
| 0.00 | 60% | 5.00 mm | 14.70% |
| 0.50 | 100% | 3.50 mm | 5.79% |
| 1.00 | 20% | 8.15 mm | 1.72% |

`alpha=0.50`은 success와 edge consistency 사이의 가장 안정적인 균형점이었다. Edge projection은 강할수록 좋은 것이 아니라, 물리적 일관성과 목표 형상 추종 사이의 절충이 필요했다.

---

## 6. 1-step MPC

1-step MPC는 각 candidate action 직후의 predicted error를 평가하고, 가장 낮은 action을 선택한다.

$$
a_t^*=\arg\min_{a\in\mathcal A(X_t)}J_1(a)
$$

구조가 단순하고 계산량이 비교적 작아 초기 teacher 수집과 중간 평가에 사용하였다.

<img src="../../../images/t2_mpc05.png" alt="Task 2 mid evaluation score" width="650"/>

<p align="center"><i>1-step MPC 기반 중간 평가에서 K-DAS는 normalized score 76.90/100으로 1위를 기록하였다.</i></p>

Top-3 average는 `0.7690`, 개별 score는 `0.8208 / 0.7653 / 0.7210`이었다. 즉 1-step MPC만으로도 기본적인 rope shape control 성능은 충분히 확보했다.

다만 이후 rollout에서는 다음 한계가 확인되었다.

- 현재 step 직후에는 좋아 보이나 다음 step에서 이어가기 어려운 action
- 목표 근처에 도달한 뒤 다시 error가 커지는 near-goal failure
- RL에 장기 수렴 관점의 teacher signal을 충분히 제공하지 못함

---

## 7. 2-step MPC

2-step MPC는 첫 action 이후의 상태에서 다시 후보 action을 평가하고, immediate cost와 future cost를 함께 고려한다.

$$
J_{\mathrm{total}}=0.3J_{\mathrm{now}}+0.7J_{\mathrm{future}}
$$

$$
a_t^*=\arg\min_{a_t}\left(J_1(a_t)+\lambda J_2(a_t)\right)
$$

Future cost에 더 높은 비중을 주어, 현재 error만 작게 만드는 action보다 다음 상태에서도 수렴하기 쉬운 action을 우선하였다.

Planning horizon을 더 길게 늘리면 탐색 비용은 대략 다음과 같이 증가한다.

$$
1\text{-step}:O(B),\qquad 2\text{-step}:O(B^2),\qquad H\text{-step}:O(B^H)
$$

각 candidate마다 DER rollout과 GNN inference가 필요하기 때문에, 2-step은 teacher quality와 계산량 사이의 현실적인 절충안이었다.

---

## 8. 1-step vs. 2-step Results

동일 seed 기반 실험에서 1-step MPC와 2-step MPC를 비교한 결과는 다음과 같다.

| Mode | Success rate | Avg. steps to success | Selected 1-step predicted error | Selected future predicted error | One-step cost $J_1$ | Future cost $J_{future}$ | Total cost $J_{total}$ | Final mean error | Best mean error |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| **1-step MPC** | 4/5 | 15.50 | 12.057 mm | 11.568 mm | 0.063959 | 0.059454 | 0.060806 | 4.408 mm | 3.751 mm |
| **2-step MPC** | **5/5** | **9.33** | **9.929 mm** | **8.984 mm** | **0.061834** | **0.054412** | **0.058725** | **3.685 mm** | **3.516 mm** |

<img src="../../../images/t2_mpc07.png" alt="Future-aware planning reduced steps-to-success" width="720"/>

2-step MPC는 단순히 horizon을 한 단계 늘린 것이 아니라, **더 빠른 수렴과 더 안정적인 teacher action**을 제공했다. 특히 success rate가 4/5에서 5/5로 증가했고, 평균 success 도달 step은 15.50에서 9.33으로 감소했다.

---

## 9. Target-edge GNN in MPC

Transition model을 기존 `exp2` GNN에서 `target-edge weight500` GNN으로 교체했을 때 2-step MPC의 closed-loop 성능도 개선되었다.

| Metric | exp2 + 2-step MPC | target-edge w500 + 2-step MPC |
|---|---:|---:|
| Success rate | 약 0.8 | 약 0.9 |
| Final mean error | 약 4.51 mm | 약 3.57 mm |
| Best mean error | 약 3.81 mm | 약 3.57 mm |
| Near-goal failure rate | 약 0.2 | 약 0.0 |

Target-edge model은 standalone position RMSE만으로는 항상 우수하지 않았지만, edge consistency가 좋아져 planner와 결합한 rollout에서는 더 안정적인 결과를 보였다. 이는 transition model을 one-step RMSE뿐 아니라 closed-loop planning 결과로도 평가해야 함을 보여준다.

---

## 10. Teacher Dataset Generation

최종 teacher dataset은 다음 설정으로 수집하였다.

| Item | Setting |
|---|---|
| GNN model | target-edge weight500 |
| Planner | 2-step MPC |
| Stroke length bins | 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.5, 0.6, 0.7, 0.8 |
| Goal edge projection | ON |
| Feasible target projection | ON |
| Candidate ranking | top-k candidate 저장 |
| Purpose | BC/RL 재학습용 teacher dataset |

여러 worker에서 JSONL을 병렬로 수집하고 통합했으며, 약 2만 개 이상의 step record를 확보하였다. Selected action뿐 아니라 shortlist candidate와 상대적 품질도 저장하여 candidate-aware RL에서 활용할 수 있게 했다.

---

## 11. Why MPC Was Not the Final Runtime Policy

MPC는 고품질 action을 생성하지만 실제 로봇에서 매 step 반복하기에는 계산량이 크다.

- 많은 candidate에 대한 반복 rollout
- 2-step expansion에 따른 계산량 증가
- 실제 로봇에서 요구되는 빠른 action decision

따라서 MPC는 **실시간 최종 controller**가 아니라 **teacher generator**로 사용되었다. 이후 Behavior Cloning과 candidate-aware offline RL이 MPC의 선택을 빠르게 근사하도록 학습되었다.

---

## 12. Limitations

- 2-step horizon은 장기적인 모든 deformation을 직접 planning하지 못함
- discrete candidate bin 때문에 continuous optimum을 직접 탐색하지 않음
- transition model error가 planning 결과에 누적될 수 있음
- projection 전후 correction magnitude를 step별로 직접 저장하지 못함
- simulation/rollout의 좋은 action이 실제 하드웨어에서도 항상 동일하게 작동하지는 않음

이 때문에 최종 시스템에는 별도의 geometry-aware runtime safety와 score hold가 필요했다.

---

## 13. Repository Structure

```text
task2/planning/mpc/
├── README.md
├── src/
│   ├── candidate_generator.py
│   ├── transition_model.py
│   ├── cost.py
│   ├── edge_projection.py
│   ├── one_step_mpc.py
│   └── two_step_mpc.py
├── configs/
├── scripts/
├── results/
```

---


> 모든 시각 자료는 repository 최상위 `images/` 폴더에서 공통 관리한다.

## 14. Takeaway

K-DAS의 MPC는 hybrid dynamics model을 실제 action selection으로 연결하는 planning layer였다. 1-step MPC는 중간 평가에서 1위를 기록할 수준의 기본 성능을 제공했으며, 2-step MPC는 future cost를 반영하여 success rate, step-to-success와 final error를 모두 개선했다.

> **MPC의 최종 역할은 느린 최적 제어기를 그대로 배포하는 것이 아니라, 이후 BC/RL이 학습할 수 있는 future-aware teacher action과 candidate ranking을 생성하는 것이었다.**
