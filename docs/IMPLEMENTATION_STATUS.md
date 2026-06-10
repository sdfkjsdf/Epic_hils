# EPIC HILS — Implementation Status

_Last updated: 2026-06-10_

Closed-loop HILS: validate that the real PX4 board (HITL) + Gazebo tracks EPIC's
trajectory, with the LiDAR-SLAM odom replaced by a Gazebo-truth stand-in
(gazebo_truth mode — no real SLAM, matching how the institution's sim also uses
ground-truth instead of LIO-SAM).

## Architecture (the closed loop)

```
  [PX4 + Gazebo HITL]
        │ /gazebo/model_states  (gazebo_msgs/ModelStates)
        ▼
  gz_truth_bridge  (SLAM stand-in: Gazebo truth -> odometry, offset 0)
        │ nav_msgs/Odometry @ /quad_0/lidar_slam/odom
        ├──────────────► pcl_render        (renders LiDAR at this pose)
        ├──────────────► EPIC (onboard)    (plans from this pose)
        └── nav_msgs/Odometry @ /mavros/odometry/out ──► PX4 EKF2 (external odom)

  [Map garage.pcd] ── sensor_msgs/PointCloud2 ──► pcl_render
  pcl_render ── sensor_msgs/PointCloud2 @ /quad0_pcl_render_node/cloud ──► EPIC

  EPIC ── traj_utils/PolyTraj @ /planning/trajectory ──► custom_trajectory_sampler
  custom_trajectory_sampler ── trajectory_msgs/MultiDOFJointTrajectory
                               @ /planning/trajectory_multidof ──► adapter
  adapter (px4_setpoint_adapter) ── mavros_msgs/PositionTarget
                                    @ /mavros/setpoint_raw/local ──► PX4
        ▲
        └──────────── PX4 flies the drone in Gazebo (loop closes) ───────────┘
```

## ✅ Implemented (done & verified)

| Item | Topic / Type | Notes |
|---|---|---|
| PX4 board HITL + Gazebo flight | — | Drone arms, takes off, hovers (verified). Firmware `pwm_out_sim`, airframe 1001, termination failsafes solved. |
| Bridge input (Gazebo state) | `/gazebo/model_states` · `gazebo_msgs/ModelStates` | Extracts iris pose+twist. |
| Bridge → odom (EPIC + pcl_render) | `/quad_0/lidar_slam/odom` · `nav_msgs/Odometry` | One odom, both subscribe. Offset 0. |
| Map → pcl_render | garage.pcd (file arg / `/map_generator/global_cloud`) · `PointCloud2` | Static map. |
| pcl_render → cloud → EPIC | `/quad0_pcl_render_node/cloud` · `sensor_msgs/PointCloud2` | MID360 sensor config matched to institution (is_360lidar=0, sensing 18, vfov 59). |
| EPIC live planning | `/planning/trajectory` · `traj_utils/PolyTraj` | Reaches EXEC_TRAJ (plan success) once airborne at ~2 m. Origin is free space; only ground (z=0.1 floor) failed. |
| Arming (EKF stable) | — | offset 0 → EKF locks to Gazebo truth → `commander arm` OK. |
| Adapter (MultiDOF → setpoint) | `/mavros/setpoint_raw/local` · `mavros_msgs/PositionTarget` | px4_setpoint_adapter consumes MultiDOFJointTrajectory, samples by time, streams PositionTarget @ 50 Hz. Code ready & syntax-verified. |

## ⏳ Implemented in code, NOT live-verified

| Item | Topic / Type | What's left |
|---|---|---|
| Bridge → PX4 external odometry | `/mavros/odometry/out` · `nav_msgs/Odometry` | Switched from vision_pose (PoseStamped) to full odometry (pose+vel, body-frame twist), EKF2_EV_CTRL=15. NOT tested: does EKF2 actually fuse it → FC position converges → arming OK? |
| EV velocity vs pose-only toggle | — | `pose_only` / `pose_vel` menu arg (EKF2_EV_CTRL 11/15) added; not exercised. |

## ❌ NOT implemented (the gap)

| Item | Topic / Type | Blocker |
|---|---|---|
| **EPIC → MultiDOF trajectory** | `/planning/trajectory_multidof` · `trajectory_msgs/MultiDOFJointTrajectory` | **custom_trajectory_sampler is NOT built.** Source exists in the container (publishes MultiDOF on `/planning/trajectory_multidof`) but is not in CMakeLists; adding it + `catkin_make` failed on a quadrotor_msgs reconfigure issue (catkin_make-built ws). Needs a clean rebuild OR build the institution's `-Simulation` workspace with `catkin build`. |
| **Closed-loop flight** | — | Drone flying EPIC's path. Blocked entirely by the sampler above (no MultiDOF → adapter has no input → no setpoints). |

## Remaining work (ordered)

1. **Build `custom_trajectory_sampler`** so `/planning/trajectory` → `/planning/trajectory_multidof` flows.
   - Recommended: build the institution's `-Simulation` workspace with `catkin build`
     (it has the sampler in CMakeLists + the official MID360 config; `catkin build` avoids
     the `catkin_make` reconfigure failure). NB topic name: sampler publishes
     `/planning/trajectory_multidof` (matches the documented multidof controller);
     point the adapter's `~traj_topic` accordingly (or remap).
2. **Live-verify the odometry bridge change**: run bring-up → confirm `/mavros/odometry/out`
   is fused by EKF2 (FC position converges to Gazebo truth) → arming OK.
3. **Close the loop**: arm → climb to ~2 m → trigger EPIC → MultiDOF → adapter → PX4
   flies EPIC's path → measure tracking error (the #4 gain-tuning goal).

## Scope notes (what is NOT ours)

- **Real SLAM (MID360 + LIO-SAM)** is the other institution's. It is NOT in either
  downloaded workspace (only its output topic names `/lio_sam/mapping/...`,
  `/odometry/imu_stamped` are referenced). On the real drone it runs as a separate
  package on the IMX8 board. In sim, both the institution and we replace it with
  ground-truth odom — so SLAM estimation error is never tested in sim.
- The HILS validates **path-tracking (PX4 + Gazebo plant)**, not perception/SLAM.
- `onboard_odom` mode is a hook: swap the bridge for real LIO-SAM odom later
  (same `nav_msgs/Odometry` interface) to test the full chain with estimation error.

## Key files (host: /home/dh/poongsan_hils)

| File | Role | Status |
|---|---|---|
| `hils_menu.py` | One-shot bring-up + test menu | ✅ (offset code removed; odometry/out check; pose_only/pose_vel arg) |
| `scripts/gz_truth_bridge.py` | SLAM stand-in (Gazebo truth → odometry) | ✅ (odometry/out, body-frame vel, offset 0) — rename to `gazebo_odom_bridge` pending |
| `scripts/configure_ekf_extodom.py` | EKF2 external-odom params | ✅ (EKF2_EV_CTRL 15, `--pose-only` flag) |
| `onboard_adapter/scripts/px4_setpoint_adapter.py` | MultiDOF → PositionTarget | ✅ (consumes MultiDOFJointTrajectory) |
| `hils_epic.launch` | pcl_render + map + EPIC planner | ✅ (MID360 config) — `custom_trajectory_sampler` node to be added |
| custom_trajectory_sampler | PolyTraj → MultiDOF | ❌ not built |
