<p align="center">
  <img src="../images/map01.png" alt="CloudGripper robot-specific mapping and matching workflow" width="980"/>
</p>

# Robot-Specific Mapping and Automatic Matching for CloudGripper

**Shared Module · Task 1 and Task 2 · Team K-DAS · RGMC 2026**

Task 1과 Task 2에서 검출되는 object, corner, rope node의 위치는 모두 camera pixel coordinate `(u, v)`로 얻어진다. 반면 CloudGripper는 normalized workspace coordinate `(x, y)`를 기준으로 움직이기 때문에, 실제 계산과 robot command를 위해서는 pixel coordinate를 normalized coordinate로 변환해야 한다.

CloudGripper는 robot마다 camera geometry와 보이는 workspace가 조금씩 달랐고, competition에서는 어느 robot에 연결되는지도 미리 알 수 없었다. K-DAS는 test period 동안 robot별 coordinate map을 미리 생성해 두고, competition 시작 시 첫 camera observation만으로 현재 robot에 맞는 map을 찾아 바로 사용하는 방식으로 이 문제를 해결했다.

> **이 문서에서 `map`은** robot command coordinate `(x, y)`와 camera에서 관측한 gripper pixel `(u, v)`의 대응 관계를 이용해 만든 **robot-specific pixel-to-workspace 변환 정보**를 의미한다.

```text
Before competition
robot identity known
→ build robot-specific maps
→ store maps for prepared robots

Competition
robot identity unknown
→ detect current gripper position
→ find the best-matching robot map
→ convert pixel coordinates during runtime
→ execute Task 1 / Task 2
```

---

## 1. Why Mapping Is Needed

Camera와 robot은 서로 다른 좌표계를 사용한다.

```text
Camera observation:  pixel coordinate (u, v)
Robot command:       normalized workspace coordinate (x, y)
```

Task 1에서는 object center, corner, contour의 pixel coordinate를 normalized coordinate로 변환한 뒤 object pose와 pushing action을 계산한다.

```text
object pixels
→ normalized coordinates
→ pose / polygon / contact point
→ push planning
```

Task 2에서는 rope segmentation으로 얻은 node들의 pixel coordinate를 normalized coordinate로 변환해 rope state와 다음 action을 계산한다.

```text
rope node pixels
→ normalized coordinates
→ rope state
→ planning / control
```

따라서 두 task 모두에서 **camera에서 측정한 pixel coordinate를 계산에 사용할 normalized workspace coordinate로 변환하는 과정**이 필요했다.

---

## 2. Why Robot-Specific Maps Were Needed

CloudGripper의 command coordinate는 공통적으로 normalized `(x, y)`를 사용하지만, 동일한 `(x, y)`가 camera image에서 나타나는 pixel 위치는 robot마다 조금씩 달랐다.

주요 원인은 robot별 camera mounting, distortion, visible workspace 차이였다. 따라서 하나의 변환식을 모든 robot에 공통으로 사용하는 대신, **각 robot에 대해 별도의 map을 생성**했다.

Test period에는 현재 연결된 robot 번호를 알고 있었기 때문에 해당 robot의 map을 미리 생성할 수 있었다. Competition에서는 robot 번호가 주어지지 않았기 때문에, 준비한 map들 중 현재 robot에 맞는 map을 자동으로 찾아야 했다.

<!-- Overview image at the top (map01) should also show that the same normalized workspace appears differently across robot cameras. -->

---

## 3. Gripper Detection: HSV to YOLOv8-Seg

Map을 생성하려면 robot을 특정 `(x, y)`로 이동시킨 뒤, camera image에서 실제 gripper center `(u, v)`를 안정적으로 찾아야 한다.

초기에는 HSV threshold를 이용해 gripper color를 분리했지만 조명, 반사, object와의 겹침에 따라 center 위치가 흔들리거나 detection이 실패하는 경우가 있었다.

최종적으로는 YOLOv8n-seg를 이용해 gripper를 segmentation하고, mask contour의 moment로 gripper center를 계산했다. 약 1,000장의 workspace image를 학습에 사용했다.

| Metric | Box | Mask |
|---|---:|---:|
| Precision | 0.995 | 0.995 |
| Recall | 1.000 | 1.000 |
| mAP50 | 0.995 | 0.995 |
| mAP50-95 | 0.930 | 0.794 |

<p align="center">
  <img src="../images/map02.png" alt="YOLOv8 segmentation and gripper centroid" width="760"/>
</p>

YOLO detector는 map 생성뿐 아니라 competition 시작 시 현재 gripper 위치를 검출해 robot map을 matching할 때도 사용했다.

---

## 4. Robot-Specific Map Generation

<p align="center">
  <img src="../images/map03.png" alt="Gripper detection at robot-specific mapping points" width="900"/>
</p>

<!-- map03: show representative gripper detections at several commanded positions -->

각 robot에 대해 gripper가 이동 가능한 workspace를 **33 × 33 grid**로 나누고, 총 1089개 위치에서 robot coordinate와 gripper pixel coordinate의 대응 관계를 측정했다.

```text
Command robot to (x, y)
→ wait until the gripper settles
→ capture camera images
→ detect gripper center (u, v)
→ save correspondence (u, v) ↔ (x, y)
→ repeat over the 33 × 33 workspace grid
```

한 지점에서는 여러 frame을 측정하고 유효한 detection들의 median center를 사용해 순간적인 detection noise를 줄였다. Camera distortion 보정도 map 생성과 runtime에서 동일하게 적용했다.

<p align="center">
  <img src="../images/map06.png" alt="33 by 33 robot-specific mapping grid" width="760"/>
</p>

<!-- map06: emphasize the 33×33 measured grid and the (u,v) ↔ (x,y) correspondence -->

결과적으로 각 robot마다 다음과 같은 measured correspondence가 만들어진다.

$$
\mathcal{D}=\{(u_i,v_i,x_i,y_i)\}_{i=1}^{N}
$$

이 correspondence와 변환 model을 저장해 이후 runtime에서 해당 robot의 map으로 불러와 사용했다.

---

## 5. Competition-Time Automatic Robot Matching

Test period와 달리 competition에서는 **현재 어느 robot에 연결되었는지 알 수 없었다.**

Competition 시작 후 robot마다 다시 33 × 33 map을 생성하는 것은 불가능하고, 몇 개의 calibration point를 새로 측정하는 것조차 task 수행 시간을 사용하게 된다.

K-DAS는 competition 시작 시 **gripper를 calibration 목적으로 움직이지 않고 현재 위치 그대로 사용**했다.

<!-- TODO: replace the flow below with ../images/map11.png: first-frame automatic robot/map matching -->

```text
Competition starts
→ robot identity is unknown
→ capture the first camera frame
→ detect the current gripper center with YOLO
→ compare it with prepared robot-specific maps
→ select the most similar robot/map
→ load the selected map
→ start the task immediately
```

즉 competition에서는 새로운 map을 만드는 대신, **미리 생성해 둔 map 중 현재 robot에 해당하는 map을 찾는 과정만 수행했다.**

이 방식으로 별도의 startup calibration 시간을 사용하지 않고 바로 Task 1 또는 Task 2를 시작할 수 있었다.

---

## 6. Runtime Pixel-to-Workspace Conversion

Competition-time matching으로 현재 robot의 map이 선택되면, 이후 Task 1과 Task 2에서는 **선택된 map을 이용해 camera에서 검출한 pixel coordinate를 normalized workspace coordinate로 변환**했다.

하지만 runtime에서 얻는 pixel `(u, v)`가 1089개의 calibration point 중 하나와 정확히 일치하는 경우는 거의 없다. 따라서 가장 가까운 한 점을 그대로 사용하는 대신, 주변 calibration point의 관계를 이용해 연속적인 좌표를 계산했다.

K-DAS는 33 × 33 correspondence로부터 Clough–Tocher 2D interpolation을 구성했다.

$$
x=f_x(u,v), \qquad y=f_y(u,v)
$$

<p align="center">
  <img src="../images/map07.png" alt="Interpolation from measured pixel-workspace correspondences" width="920"/>
</p>

Runtime에서는 선택된 robot map을 load한 뒤, 측정된 calibration point 사이를 보간해 `(x, y)`를 계산했다.

```text
Detected object / rope pixel (u, v)
→ load selected robot map
→ interpolate between measured calibration points
→ normalized workspace coordinate (x, y)
→ Task 1 / Task 2 calculation
```

Repository에서는 저장된 변환 model을 편의상 `LUT`라고 부르지만, 실제 변환은 단순 nearest-neighbor lookup이 아니라 **측정한 1089개 point 사이를 interpolation하여 연속적인 `(x, y)`를 계산하는 과정**이다.

---

## 7. Extending Coordinates Beyond the Reachable Workspace

33 × 33 map은 gripper가 실제로 이동할 수 있는 영역에서만 측정할 수 있다. 그런데 camera에서 보이는 object나 rope가 항상 그 영역 안에 완전히 들어오는 것은 아니었다.

예를 들어 square object의 네 corner 중 세 점은 mapping 영역 안에 있지만 한 점이 바깥에 있을 수 있다.

```text
object corners:  ●──────●
                 │      │
workspace edge: ─●──────┼────────
                        ●  ← outside measured region
```

이 경우 바깥쪽 corner는 LUT의 측정 범위를 벗어나기 때문에 정상적인 normalized coordinate를 얻을 수 없다. Rope 역시 일부 node가 workspace 밖에 있을 경우 전체 shape을 표현하기 어려워진다.

### 7.1 Homography from the 33 × 33 map

이 문제를 해결하기 위해 기존 33 × 33 map에서 얻은 `(x, y) ↔ (u, v)` correspondence를 이용해 homography를 계산했다.

$$
\mathbf{p}_{uv} \sim H_{xy\rightarrow uv}\mathbf{p}_{xy}
$$

Inverse homography를 이용하면 camera pixel에서 확장된 normalized coordinate를 계산할 수 있다.

$$
\mathbf{p}_{xy}^{ext} \sim H_{uv\rightarrow xy}\mathbf{p}_{uv}
$$

<p align="center">
  <img src="../images/map10.png" alt="Reachable workspace and homography-based extended coordinates" width="900"/>
</p>

Homography를 이용하면 gripper가 직접 갈 수 없는 영역에 있는 object corner나 rope node도 하나의 coordinate system으로 표현할 수 있다.

### 7.2 How We Checked the Homography

Workspace 바깥에는 gripper를 직접 이동시킬 수 없기 때문에 homography가 만든 outside coordinate를 직접 측정해 검증할 방법이 없었다.

그래서 **LUT와 homography를 모두 계산할 수 있는 workspace 내부 영역에서 두 결과를 비교**했다.

```text
Same pixel inside the measured workspace
        ├─ LUT interpolation → xy_lut
        └─ Homography        → xy_homo
                         ↓
                    compare error
```

내부 영역에서 두 좌표의 차이가 충분히 작음을 확인한 뒤, object / rope의 state coordinate는 inside와 outside에서 서로 다른 변환을 섞지 않고 homography 기반 coordinate로 통일해 사용했다.

단, workspace 바깥의 coordinate는 **state를 표현하기 위한 값**이며 실제 robot이 이동할 수 있는 command coordinate라는 의미는 아니다. Robot action을 생성할 때는 reachable workspace 여부를 별도로 확인했다.

---

## 8. Validation

### 8.1 Off-grid Mapping Validation

33 × 33 mapping point 자체에서 다시 검사하면 interpolation이 잘 만들어졌는지만 확인할 수 있다. 실제 변환 성능을 확인하기 위해 grid와 겹치지 않는 새로운 robot coordinate를 사용했다.

```text
Move gripper to an off-grid coordinate
→ detect the gripper pixel
→ convert the pixel using the saved map
→ compare mapped coordinate with the commanded coordinate
```

한 validation run에서는 다음 결과를 얻었다.

| Metric | Result |
|---|---:|
| Valid samples | 25 / 25 |
| Mean error | 0.272 mm |
| Median error | 0.303 mm |
| Maximum error | 0.522 mm |

<p align="center">
  <img src="../images/map08.png" alt="Off-grid pixel-to-workspace validation" width="720"/>
</p>

### 8.2 Homography Overlap Check

Homography의 outside 영역은 직접 ground truth를 측정할 수 없기 때문에, workspace 내부에서 LUT interpolation과 homography coordinate를 비교했다. 이 검증은 outside accuracy를 직접 보장하는 것은 아니지만, measured workspace 안에서 두 방식이 얼마나 비슷한 coordinate를 만드는지 확인하는 기준으로 사용했다.

---

## 9. Use in Task 1 and Task 2

<!-- TODO: add a combined Task 1 / Task 2 application figure (e.g. ../images/map12.png) showing detected pixels → normalized coordinates → planning input -->

### Task 1 — Rigid Object Pushing

```text
camera image
→ object segmentation / contour
→ object pixel coordinates
→ normalized coordinates
→ pose / polygon / contact calculation
→ push planning
```

물체 일부가 measured workspace 밖에 걸치는 경우에는 homography coordinate를 이용해 전체 polygon state를 유지하고, 실제 robot action은 reachable workspace 안에서만 생성했다.

### Task 2 — Rope Manipulation

```text
camera image
→ rope segmentation / skeleton
→ ordered rope-node pixels
→ normalized coordinates
→ rope state
→ DER / Residual GNN / MPC / policy
```

Rope는 전체 node의 배치가 state이기 때문에 일부 node가 workspace 밖으로 나가더라도 삭제하지 않고 homography coordinate로 함께 표현했다.

---

## 10. Summary

K-DAS의 mapping 과정은 다음 세 가지 문제를 해결하기 위해 구성되었다.

1. **Map generation** — robot별 33 × 33 pixel–workspace correspondence를 미리 측정하고 interpolation map을 생성한다.
2. **Automatic map matching** — competition 시작 시 첫 gripper observation을 이용해 현재 robot에 해당하는 map을 찾는다.
3. **Runtime coordinate conversion** — 선택된 map과 homography를 이용해 Task 1 object와 Task 2 rope의 pixel coordinate를 normalized coordinate로 변환한다.

```text
Generate maps before competition
→ identify the connected robot at startup
→ load its map
→ convert camera pixels during runtime
→ use the converted coordinates for Task 1 / Task 2
```
