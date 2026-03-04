# RSU Solver Node 

상위 제어기에서 **양발 발목(RSU 2DOF)**에 대한 **Roll/Pitch 목표값을 입력으로 받아**, 각 발의 **Actuator 2개(총 4개)** 해를 계산해주는 **IK 컴포넌트 노드**입니다.

---

## What it does

- 입력: `left/right roll, pitch` 목표 (rad)
- 출력: `left/right actuator_1, actuator_2` (rad)
- 특징:
    - continuity: 이전 해(`prev_alpha`) 기반으로 해의 branch continuity 유지
    - infeasible 처리: infeasible이면 이전 actuator 값을 유지(옵션)

---

## Topics

### Subscribe

- `/rsu/target` (`roa_interfaces/msg/RsuTarget`)
    - `left_roll`, `left_pitch`, `right_roll`, `right_pitch` (float32, rad)
    - `seq` + `header.stamp` 기반으로 out-of-order 메시지 drop

### Publish

- `/rsu/solution` (`roa_interfaces/msg/RsuSolution`)
    - `left_actuator_1`, `left_actuator_2`, `right_actuator_1`, `right_actuator_2` (float32, rad)
    - `feasible` (bool): **이번 요청에서 좌/우 모두 feasible이면 true**

(디버그 모드에서만)

- `/joint_states` (`sensor_msgs/msg/JointState`)
- `/solver_answer` (`geometry_msgs/msg/Vector3Stamped`)
- `/request_to_solver` (`geometry_msgs/msg/Vector3Stamped`) 구독

---

## Parameters

필수/주요 파라미터만 정리합니다.

### Mode

- `REALTIME_CONTROL_MODE` (bool, default: `false`)
    - `true`: RT 모드(`/rsu/target` ↔ `/rsu/solution`)
    - `false`: 디버그 모드(조이패드/JointState/TF 체크)

### IK behavior

- `hold_alpha_on_infeasible` (bool, default: `true`)
    - infeasible이면 actuator 값을 이전 값으로 hold

### RSU Geometry

- `a_W_mm_flat` (float[6])
- `b_F_mm_flat` (float[6])
- `c_mm` (float[2])
- `r_mm` (float[2])
- `psi_rad` (float[2])

### (Debug) TF sanity check

- `gate_publish_by_tf_check` (bool, default: `true`)
- `world_frame`, `c1_frame`, `u1_frame`, `c2_frame`, `u2_frame`
- `tf_timeout_sec`, `len_tol_m`, `ang_min_deg`, `ang_max_deg`

---

## Build

```
colcon build--symlink-install
source install/setup.bash
```

---

## Run

### Realtime Control Mode (recommended for controller integration)

```
ros2 run <your_pkg> rsu_solver_node.py--ros-args-p REALTIME_CONTROL_MODE:=true
```

### Debug Mode (for validation)

```
ros2 run <your_pkg> rsu_solver_node.py--ros-args-p REALTIME_CONTROL_MODE:=false
```

---

## Integration Contract (Controller View)

상위 제어기는 다음만 지키면 됩니다.

1. `/rsu/target`에 roll/pitch를 **rad**로 publish
2. `seq` 또는 `header.stamp`가 **단조 증가**하도록 유지
3. `/rsu/solution`의 actuator 값을 받아 하드웨어/ros2_control로 전달
4. `feasible==false`면 해당 step은 “완전 유효해 아님”으로 처리(hold/감속/중단 등 상위 정책 적용)

---

## Notes

- 좌우 발이 완전 미러 구조인 경우, 상위 제어기에서 roll/actuator sign을 맞춰주는 전략을 사용할 수 있습니다.
- TF 기반 sanity check는 디버그에 유용하지만, RT 루프에서는 블로킹/지연 요인이 될 수 있으므로 운용 정책에 맞게 사용하세요.
