# Coordinate alignment - MARSIM/EPIC (ENU world) <-> Gazebo/PX4

> The single tricky thing that MUST be aligned in HITL live mode.
> If misaligned: the MARSIM point cloud lands in the wrong place (EPIC misbehaves),
> and the drone flies to a different position than commanded.

## 1. Frame conventions per system
| System | Frame | Note |
|--------|-------|------|
| EPIC / MARSIM | ENU world (x=East, y=North, z=Up) | odom (/lidar_slam/odom) and point cloud are in this world |
| PX4 (internal) | NED | internal estimation/control is NED |
| MAVROS (exposed) | ENU local | /mavros/local_position/odom, setpoint_raw/local are ENU; MAVROS does NED<->ENU |
| Gazebo | own world (usually ENU) | origin/heading depend on the model spawn pose |

EPIC and MAVROS are both ENU, so axes match. The remaining issue is the origin and initial yaw offset.

## 2. Why it matters
- Point cloud misalignment: MARSIM renders the LiDAR from a given pose. If the Gazebo odom
  origin/yaw differs from the MARSIM world, the cloud is generated at the wrong place/orientation,
  so EPIC sees fake obstacles/free space and plans incorrectly.
- Command misalignment: EPIC /position_cmd is in the EPIC world. If the bridge passes it to the
  PX4/MAVROS ENU local frame with an origin/yaw offset, the drone flies to the wrong absolute
  position, making the tracking-error measurement meaningless.

## 3. Three things to align
1. Origin: map the EPIC world origin (MARSIM map reference / drone start) to the same point in Gazebo/PX4 local.
2. Yaw (initial heading): make 0 deg (East) point the same way in all three.
3. Frame convention: ENU vs NED is handled by MAVROS; just verify the axes (x=E, y=N, z=U).

## 4. Where to align (implementation)
- Strategy A - unify the origin (recommended, simplest): start every system at (0,0,0), yaw=0.
  - Spawn the Gazebo drone at origin / yaw=0.
  - Set the PX4 EKF origin accordingly.
  - Use the MARSIM map reference / drone start at the origin.
  - => zero offset => no transform node; odom/setpoint pass through 1:1.
- Strategy B - transform node: if an offset is unavoidable, apply an SE(3) offset (translation + yaw):
  - Gazebo odom -> EPIC world odom
  - /position_cmd -> PX4 setpoint frame (inverse)
  - Best placed inside the bridge node (/position_cmd -> setpoint).

## 5. How to verify
1. Static alignment: with the drone at both origins, EPIC odom and /mavros/local_position/odom should both read near (0,0,z).
2. Single-axis move: send a known setpoint (+1m East); confirm Gazebo moves East and EPIC odom x increases by 1.
3. Trajectory overlay: fly briefly and overlay /position_cmd (reference) vs /mavros/local_position/odom (actual);
   aligned frames differ only by the tracking error, with matching direction/scale.

## 6. Default decision
Start with Strategy A (all origin=0, yaw=0, same ENU). Add a Strategy B transform in the bridge
only if an offset shows up.

---
### Related
- Interface: EPIC -> /position_cmd (quadrotor_msgs::PositionCommand) / HILS -> /lidar_slam/odom (nav_msgs::Odometry)
- The point cloud is rendered by poongsan's (epic_planner) MARSIM from the odom pose (not re-created).
