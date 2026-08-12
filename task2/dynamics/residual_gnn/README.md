<img src="../../../images/t2_gnn01.png" alt="DER and Residual GNN hybrid dynamics pipeline" width="900"/>

# Residual GNN for Rope Dynamics Correction

**Task 2 · Linear Deformable Object Shape Control · Team K-DAS**

이 모듈은 DER 기반 물리 모델이 예측한 다음 로프 상태와 실제 로봇 결과 사이의 **systematic residual error**를 Graph Neural Network로 보정한다. 로프를 20개의 ordered node로 구성된 chain graph로 표현하고, 현재 상태·DER rollout·grasp action 정보를 이용해 각 node의 2D residual displacement를 예측한다.

> 본 구현은 GraphDLO의 문제 설정과 graph-based dynamics prediction 개념에서 영감을 받았지만, 논문의 전체 architecture를 그대로 복제한 것이 아니다. K-DAS의 DER baseline, 20-node state representation, CloudGripper action format, MPC teacher pipeline에 맞게 구성한 **GraphDLO-inspired residual correction model**이다.

---

## 1. Why Residual Learning?

DER는 bending, damping, fixed-end, edge-length preservation과 같은 구조적 prior를 제공했지만, 실제 로프의 세부 거동에는 다음 요소가 함께 작용했다.

- 로프 재질과 개체별 차이
- Table friction과 stick-slip
- Gripper의 접촉 위치와 jaw interaction
- Drag speed와 release dynamics
- Segmentation 및 20-node 추출 오차
- 고정단에서 멀어질수록 커지는 endpoint propagation uncertainty

이 모든 현상을 물리 모델 하나에서 정확하게 식별하기는 어려웠다. 반대로 다음 상태 전체를 pure learning model로 직접 예측하면, 제한된 데이터에서 로프 길이와 연결 구조가 무너질 수 있다.

따라서 질문을 다음과 같이 바꾸었다.

> **“DER의 물리적 구조는 유지하면서, 남아 있는 예측 오차만 학습 기반으로 보정할 수 있는가?”**

Residual learning은 GNN이 전체 dynamics를 처음부터 다시 배우는 대신, DER가 설명하지 못한 부분에 집중하도록 만든다.

---

## 2. Role in the Full Task 2 System

```text
Current rope X_t + Action a_t
             │
             ├── Planar DER rollout ──> X̂_DER,t+1
             │
             └── Action-conditioned graph features
                              │
                       Residual GNN
                              │
                       ΔX̂_GNN,t+1
                              │
                              ▼
        X̂_t+1 = X̂_DER,t+1 + ΔX̂_GNN,t+1
                              │
                              ▼
                   MPC candidate evaluation
```

Residual GNN은 최종 action을 직접 출력하는 policy가 아니다. 이 모듈은 **transition model**의 한 부분이며, 보정된 다음 상태는 1-step/2-step MPC의 candidate evaluation과 teacher dataset 생성에 사용된다.

관련 문서:

- Task 2 overview: [`../../README.md`](../../README.md)
- DER baseline: [`../der/README.md`](../der/README.md)
- MPC planner: [`../../planning/mpc/README.md`](../../planning/mpc/README.md)
- BC/RL policy: [`../../policy/candidate_aware_rl/README.md`](../../policy/candidate_aware_rl/README.md)

---

## 3. Hybrid Dynamics Formulation

현재 상태와 action으로 DER prediction을 생성한다.

$$
\hat{X}_{t+1}^{\mathrm{DER}} = f_{\mathrm{DER}}(X_t,a_t)
$$

Ground-truth transition에서 DER가 설명하지 못한 residual target은 다음과 같다.

$$
\Delta X_{t+1}^{\mathrm{GT}} = X_{t+1}^{\mathrm{GT}}-\hat{X}_{t+1}^{\mathrm{DER}}
$$

GNN은 residual을 예측한다.

$$
\Delta \hat{X}_{t+1}=g_\theta(G_t,a_t,\hat{X}_{t+1}^{\mathrm{DER}})
$$

최종 prediction은 DER output과 residual correction의 합이다.

$$
\hat{X}_{t+1}=\hat{X}_{t+1}^{\mathrm{DER}}+\Delta \hat{X}_{t+1}
$$

이 구조의 장점은 다음과 같다.

1. DER가 제공하는 fixed-end, local connectivity, 기본 변형 전달을 유지한다.
2. GNN의 target magnitude가 전체 node position보다 작은 residual로 제한된다.
3. 물리 모델과 실제 환경 사이에서 반복되는 bias를 데이터로 학습할 수 있다.
4. Physics model과 learned correction을 독립적으로 진단할 수 있다.

---

## 4. Rope Graph Representation

<img src="../../../images/t2_gnn02.png" alt="20-node rope graph and action-conditioned features" width="900"/>

로프 상태는 다음 chain graph로 표현한다.

$$
G=(V,E), \qquad |V|=20
$$

$$
E=\{(i,i+1),(i+1,i)\mid i=0,\ldots,18\}
$$

- **Node**: ordered rope point
- **Edge**: 물리적으로 인접한 rope segment
- **Node index direction**: fixed end에서 free end 방향
- **Message passing**: grasp에서 발생한 변형 정보가 인접 node를 따라 전달되도록 모델링

Chain graph를 사용하면 fully connected network보다 로프의 실제 연결 구조를 직접 반영할 수 있다. 또한 node 수가 고정된 20개이므로, 각 prediction은 `[20, 2]` 형태의 residual displacement를 출력한다.

---

## 5. Input and Output Features

프로젝트에서 사용한 입력 정보는 다음 범주로 구성된다.

### 5.1 Node-local information

- 현재 node 좌표 `X_t[i]`
- DER가 예측한 node 좌표 `X_DER[i]`
- DER displacement `X_DER[i] - X_t[i]`
- 정규화된 node index
- grasp node 여부를 나타내는 flag

### 5.2 Action context

- grasp point
- target point
- drag vector
- 이동 길이
- 선택된 grasp node index

Action 정보는 grasp node에서만 의미가 있는 feature와 전체 node에 broadcast되는 global context로 나누어 사용할 수 있다. 공개 코드에서는 실제 tensor 구성 순서와 normalization을 configuration 또는 dataset 문서에 명확히 고정해야 한다.

### 5.3 Output

각 node의 2D residual displacement를 출력한다.

$$
\Delta \hat{X}_{t+1}\in\mathbb{R}^{20\times2}
$$

Fixed-end node는 후처리 또는 mask를 통해 이동하지 않도록 유지한다.

---

## 6. Network Structure

전체 구조는 다음 세 단계로 설명할 수 있다.

1. **Node/Action Encoder**  
   현재 state, DER prediction과 action context를 hidden representation으로 변환한다.
2. **Graph Message Passing**  
   인접 node 사이에서 정보를 교환하여 local deformation propagation을 학습한다.
3. **Residual Decoder**  
   각 node hidden state에서 `(Δx, Δy)`를 회귀한다.

```python
node_hidden = node_encoder(node_features, action_context)

for layer in message_passing_layers:
    node_hidden = layer(node_hidden, edge_index)

residual_xy = residual_decoder(node_hidden)   # [20, 2]
predicted_next = der_next + residual_xy
```

이 README는 구조적 역할을 설명하는 문서다. 실제 공개 시에는 layer 수, activation, normalization, dropout, optimizer와 checkpoint format을 코드 및 config에서 고정해야 한다.

---

## 7. Dataset and Cache Pipeline

각 학습 sample은 실제 robot transition으로 구성된다.

$$
(X_t,a_t,X_{t+1}^{\mathrm{GT}})
$$

원본 JSONL을 학습할 때마다 읽고 DER rollout을 반복하면 큰 병목이 발생한다. 이를 해결하기 위해 `RopeCachedDataset` 구조를 사용하였다.

```text
Raw JSONL transitions
        │
        ├── validate node ordering / shape
        ├── normalize coordinates and actions
        ├── run DER baseline once
        ├── compute residual target
        ▼
Serialized cache file
        │
        ▼
RopeCachedDataset → DataLoader → GNN training
```

Cache에는 최소한 다음 항목이 포함되어야 한다.

- current rope state
- action representation
- DER next-state prediction
- ground-truth next state
- residual target
- edge index
- normalization metadata

이 구조는 DER rollout을 epoch마다 다시 수행하지 않으므로 hidden dimension, loss weight와 network configuration의 반복 실험을 빠르게 만들었다.

---

## 8. Training Objective Evolution

### 8.1 Node-wise residual / position loss

초기 목적은 node 위치 prediction을 개선하는 것이었다.

$$
L_{\mathrm{pos}}=\frac{1}{N}\sum_{i=0}^{N-1}\left\|\hat{x}_{i,t+1}-x_{i,t+1}^{\mathrm{GT}}\right\|_2^2
$$

Equivalent residual target loss로 표현하면 다음과 같다.

$$
L_{\mathrm{res}}=\frac{1}{N}\sum_i\left\|\Delta\hat{x}_{i,t+1}-\Delta x_{i,t+1}^{\mathrm{GT}}\right\|_2^2
$$

초기 모델은 node 위치 오차를 크게 줄였지만, 일부 prediction에서 edge가 비정상적으로 늘어나거나 줄어드는 문제가 남았다.

### 8.2 Edge consistency diagnosis

Edge length는 다음과 같이 계산한다.

$$
l_i=\|x_{i+1}-x_i\|_2
$$

평균 상대 edge error는 다음과 같다.

$$
E_{\mathrm{edge}}=\frac{1}{N-1}\sum_i\frac{|l_i^{\mathrm{pred}}-l_i^{\mathrm{target}}|}{l_i^{\mathrm{target}}+\epsilon}
$$

한 분석에서는 DER 이후 평균 edge error가 5.98%였지만, 기존 residual correction 이후 15.88%까지 증가하였다. 이 결과는 position loss만으로는 rope length consistency가 자동으로 보존되지 않는다는 점을 보여주었다.

### 8.3 Target-edge loss

초기 edge regularization을 강화하면서, 모든 edge를 하나의 일정한 rest length로 강제하는 대신 **각 sample의 실제 target state가 가진 edge length 분포**를 사용하였다.

```python
target_edges = torch.norm(y_gt[:, 1:, :] - y_gt[:, :-1, :], dim=-1)
pred_edges   = torch.norm(pred_full[:, 1:, :] - pred_full[:, :-1, :], dim=-1)
edge_loss    = ((pred_edges - target_edges) ** 2).mean()
```

$$
L_{\mathrm{edge}}=\frac{1}{N-1}\sum_i\left(l_i^{\mathrm{pred}}-l_i^{\mathrm{GT}}\right)^2
$$

최종 loss는 개념적으로 다음과 같다.

$$
L_{\mathrm{total}}=L_{\mathrm{pos}}+\lambda_{\mathrm{edge}}L_{\mathrm{edge}}
$$

`λ_edge`를 100, 500, 1000 수준에서 비교한 결과, weight 500이 edge consistency 개선과 position accuracy 사이에서 가장 적절한 기준 모델로 선택되었다.

---

## 9. Hidden Dimension Exploration

<img src="../../../images/t2_gnn03.png" alt="Initial validation performance for DER and GNN hidden dimensions" width="760"/>

Hidden dimension은 `64, 256, 1024, 4096` 후보를 탐색했다. 대표 validation에서는 1024-dimensional model이 4096 model보다 더 좋은 결과를 기록했다.

| Model | Mean RMSE | Mean MAE | Mean node L2 |
|---|---:|---:|---:|
| DER baseline | 7.67 mm | 6.24 mm | 9.60 mm |
| Residual GNN, hidden 1024 | **2.41 mm** | **1.92 mm** | **3.02 mm** |
| Residual GNN, hidden 4096 | 3.36 mm | 2.71 mm | 4.31 mm |

1024 model은 DER baseline 대비 RMSE를 5.26 mm 감소시켰다. 4096 model의 성능이 더 낮았다는 결과는 model capacity를 무조건 늘리는 것이 항상 유리하지 않으며, 데이터 규모와 regularization을 함께 고려해야 한다는 점을 보여준다.

> 이 표는 동일 순서의 validation sample 10개를 사용한 초기 비교 결과다. 이후 exp2/target-edge 실험은 다른 dataset과 protocol을 사용했으므로 수치를 직접 혼합해서는 안 된다.

---

## 10. Target-Edge Model Results

<img src="../../../images/t2_gnn04.png" alt="Position accuracy and edge consistency trade-off" width="800"/>

후속 실험에서는 위치 정확도가 우수한 `exp2 residual GNN`과 target-edge weight500 모델을 비교하였다.

| Model | Position RMSE | Edge relative error | Total length relative error | Interpretation |
|---|---:|---:|---:|---|
| exp2 residual GNN | **3.07 mm** | 12.9% | 3.1% | Node position은 정확하지만 edge consistency가 약함 |
| target-edge weight500 | 4.89 mm | **3.1%** | **0.77%** | 위치 RMSE는 증가했지만 rope length consistency가 크게 개선됨 |

이 결과는 단일 RMSE만으로 dynamics model을 선택하면 안 된다는 점을 보여준다. Rope manipulation에서 예측된 point cloud가 target에 가까워도, edge length가 크게 왜곡되면 후속 rollout과 action ranking이 비현실적으로 변할 수 있다.

따라서 최종 teacher generation에는 target-edge weight500 model을 사용하였다.

---

## 11. Downstream MPC Effect

<img src="../../../images/t2_gnn05.png" alt="Downstream 2-step MPC effect of target-edge GNN" width="900"/>

Target-edge model은 standalone validation에서 position RMSE가 더 높았지만, 2-step MPC closed-loop에서는 더 좋은 결과를 보였다.

| Metric | exp2 + 2-step MPC | target-edge w500 + 2-step MPC |
|---|---:|---:|
| Success rate | 약 0.8 | **약 0.9** |
| Final mean error | 약 4.51 mm | **약 3.57 mm** |
| Best mean error | 약 3.81 mm | **약 3.57 mm** |
| Near-goal failure rate | 약 0.2 | **약 0.0** |

이는 transition model을 평가할 때 one-step prediction RMSE뿐 아니라 **closed-loop planning 결과**도 확인해야 한다는 의미다. Edge-consistent prediction은 다음 action candidate를 평가하는 과정에서 더 안정적인 state를 제공했다.

---

## 12. GNN Loss vs. Edge Projection

Target-edge loss와 edge projection은 서로 다른 단계에서 작동한다.

| Component | Stage | Function |
|---|---|---|
| Target-edge loss | GNN training | Raw GNN prediction 자체가 target edge distribution을 보존하도록 학습 |
| Goal edge projection | MPC rollout 후처리 | Predicted state를 target edge length에 가까워지도록 기하학적으로 보정 |

Projection OFF 조건에서 final edge relative error는 exp2가 약 0.103, target-edge w500이 약 0.052로 감소했다. Projection ON에서는 두 모델 모두 edge error가 거의 0에 가까워졌기 때문에, projection이 GNN의 raw 차이를 가렸다.

따라서 target-edge GNN의 의미는 projection을 대체하는 것이 아니라, **projection이 수정해야 하는 원래 prediction의 왜곡을 줄이는 것**이다. Projection alpha에 대한 전체 ablation은 MPC 문서에서 다룬다.

---

## 13. Inference Procedure

```python
def predict_next_state(current_nodes, action):
    der_next = der_rollout(
        current_nodes=current_nodes,
        grasp_node=action.node_index,
        target_position=action.target_xy,
    )

    graph = build_graph_features(
        current_nodes=current_nodes,
        der_next=der_next,
        action=action,
    )

    residual = residual_gnn(graph)       # [20, 2]
    predicted_next = der_next + residual

    predicted_next[0] = current_nodes[0] # fixed endpoint
    return predicted_next
```

MPC에서는 이 함수를 shortlist candidate마다 반복 호출한다. 따라서 inference speed, batch evaluation과 deterministic preprocessing이 중요하다.

---

## 14. What Worked

- DER baseline 대비 세부 node prediction 오차를 크게 감소시켰다.
- 20-node chain graph를 통해 local deformation propagation을 표현했다.
- Residual target을 사용해 learning scope를 물리 모델의 오차로 제한했다.
- Cache dataset으로 반복 실험의 병목을 줄였다.
- Target-edge loss로 raw prediction의 rope-length consistency를 개선했다.
- Edge-consistent GNN을 사용했을 때 2-step MPC rollout 성능도 개선되었다.

---

## 15. Limitations

| Limitation | Effect |
|---|---|
| Dataset/domain dependence | 특정 rope, workspace와 action distribution에서 학습된 correction이 다른 재질에 그대로 일반화되지 않을 수 있음 |
| Perception noise | Ground truth node 자체가 segmentation, skeletonization과 mapping error를 포함함 |
| Endpoint uncertainty | Free endpoint는 작은 contact 차이에도 큰 displacement가 발생하여 예측 분산이 큼 |
| One-step residual model | 장기 rollout에서 작은 bias가 누적될 수 있음 |
| Fixed topology | 20-node ordering이 깨지거나 self-crossing에서 skeleton ordering이 잘못되면 graph input도 잘못됨 |
| RMSE–physics trade-off | 위치 RMSE와 edge consistency를 동시에 최적화하기 위한 loss weight가 필요함 |
| Projection dependency | MPC에서 projection을 강하게 사용하면 raw model quality가 평가 지표에서 가려질 수 있음 |

Thicker red rope에서 최종 성능이 상대적으로 낮았던 점은 dynamics correction의 domain generalization과 runtime execution을 함께 개선해야 한다는 과제로 남았다.

---

## 16. Recommended Repository Structure

```text
task2/dynamics/residual_gnn/
├── README.md
├── src/
│   ├── model.py
│   ├── graph_features.py
│   ├── dataset.py
│   └── inference.py
├── configs/
│   ├── exp2.yaml
│   └── target_edge_w500.yaml
├── scripts/
│   ├── build_cache.py
│   ├── train.py
│   ├── evaluate.py
│   └── compare_edge_metrics.py
├── models/
│   └── README.md
├── notebooks/
│   └── visualize_prediction.ipynb
├── results/
```

공개 전 반드시 고정해야 할 항목:

- Node feature 순서와 dimension
- Coordinate normalization과 workspace scale
- Edge index 방향
- Fixed endpoint mask
- DER checkpoint/config dependency
- Hidden dimension과 message-passing layer 설정
- `λ_edge` 및 target-edge definition
- Train/validation split 기준
- Cache schema/version
- Model weight checksum

---


> 모든 시각 자료는 repository 최상위 `images/` 폴더에서 공통 관리한다.

## 17. Takeaway

> **The Residual GNN preserves the DER physical prior while learning the systematic node-level correction required by the real rope.**

K-DAS의 Residual GNN은 DER를 제거하고 pure neural dynamics로 대체한 모델이 아니다. DER가 제공하는 기본 구조 위에서 실제 로프의 friction, contact, endpoint propagation과 perception uncertainty로 인해 남는 residual을 graph message passing으로 학습하였다. 초기 모델은 node RMSE를 크게 줄였고, 이후 target-edge loss를 도입하여 위치 정확도와 rope-length consistency의 균형을 개선했다. 이 hybrid transition model은 최종적으로 2-step MPC teacher와 BC/RL policy를 구축하는 기반이 되었다.

---

## References

1. H. Dinkel et al., “GraphDLO: Graph-Based Neural Dynamics for Deformable Linear Object Trajectory Prediction,” 2025. [Paper](https://florianpokorny.com/static/publications/dinkel2025a.pdf)
2. M. Bergou et al., “Discrete Elastic Rods,” ACM Transactions on Graphics, 2008.
3. K-DAS Task 2 project documentation: [`../../README.md`](../../README.md)
