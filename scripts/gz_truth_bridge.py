#!/usr/bin/env python3
"""
gz_truth_bridge.py - SIM SLAM stand-in: publish ONE Gazebo ground-truth odometry
to both EPIC and the FC, with NO offset, using the FULL nav_msgs/Odometry
(pose + velocity) - exactly what a LiDAR SLAM (LIO-SAM) gives.

Why odometry (not vision_pose): PX4 docs - "the ODOMETRY message is the only
message that can send also linear velocities to PX4" and is the preferred external
estimate for LiDAR/VIO with EKF2. The real drone's LIO-SAM outputs nav_msgs/Odometry
(pose + twist); we feed that SAME message straight into PX4 (no down-convert to a
pose-only vision estimate), and the same odometry into EPIC. One source, one format.

Reads the true pose+twist of the drone model from Gazebo (/gazebo/model_states) and
republishes nav_msgs/Odometry (offset 0) to:
  - ~odom_topic (default /quad_0/lidar_slam/odom)  -> EPIC odom input
  - /mavros/odometry/out                           -> PX4 EKF2 external odometry
                                                      (MAVLink ODOMETRY, pose+vel)

Frames: pose in ENU parent frame; twist (linear+angular) in the BODY (child) frame
per the nav_msgs/Odometry convention - PX4 requires the velocity in body frame, so
we rotate Gazebo's world-frame twist into the body frame. child_frame_id="base_link".

NOTE: requires (a) the MAVROS 'odometry' plugin enabled, and (b) EKF2 to fuse EV
velocity - set EKF2_EV_CTRL to include the velocity bit (15 = pos+vel+yaw), see
scripts/configure_ekf_extodom.py.

Params:
  ~model_name (str,   default "iris")
  ~rate       (float, default 30.0) [Hz]
  ~frame_id   (str,   default "odom")  parent frame of the published odometry
  ~odom_topic (str,   default "/quad_0/lidar_slam/odom")  EPIC odom topic
  ~fcu_topic  (str,   default "/mavros/odometry/out")     PX4 external-odom topic
  ~also_vision(bool,  default False)   also publish /mavros/vision_pose/pose (fallback)
"""
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def _world_to_body(vx, vy, vz, q):
    """Rotate a vector from the world (ENU) frame into the body frame, given the
    body->world orientation quaternion q. v_body = R(q)^T * v_world."""
    x, y, z, w = q.x, q.y, q.z, q.w
    # rotation matrix R (body->world)
    r00 = 1 - 2 * (y * y + z * z); r01 = 2 * (x * y - z * w); r02 = 2 * (x * z + y * w)
    r10 = 2 * (x * y + z * w); r11 = 1 - 2 * (x * x + z * z); r12 = 2 * (y * z - x * w)
    r20 = 2 * (x * z - y * w); r21 = 2 * (y * z + x * w); r22 = 1 - 2 * (x * x + y * y)
    # R^T * v_world  (columns of R become rows)
    bx = r00 * vx + r10 * vy + r20 * vz
    by = r01 * vx + r11 * vy + r21 * vz
    bz = r02 * vx + r12 * vy + r22 * vz
    return bx, by, bz


class GtBridge:
    def __init__(self):
        self.model = rospy.get_param("~model_name", "iris")
        self.rate = float(rospy.get_param("~rate", 30.0))
        self.frame = rospy.get_param("~frame_id", "odom")
        self.odom_topic = rospy.get_param("~odom_topic", "/quad_0/lidar_slam/odom")
        self.fcu_topic = rospy.get_param("~fcu_topic", "/mavros/odometry/out")
        self.also_vision = bool(rospy.get_param("~also_vision", False))
        self.pose = None      # latest geometry_msgs/Pose (Gazebo truth, no offset)
        self.twist = None     # latest geometry_msgs/Twist (Gazebo = WORLD frame)
        self.warned = False

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)   # EPIC
        self.fcu_pub = rospy.Publisher(self.fcu_topic, Odometry, queue_size=10)      # PX4
        self.vis_pub = (rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=10)
                        if self.also_vision else None)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._cb, queue_size=1)
        rospy.loginfo("[gt_bridge] SLAM stand-in (offset 0, full odometry): model=%s rate=%.0fHz "
                      "-> %s (EPIC) + %s (PX4 EKF2)%s",
                      self.model, self.rate, self.odom_topic, self.fcu_topic,
                      " + vision_pose" if self.also_vision else "")

    def _cb(self, msg):
        try:
            i = msg.name.index(self.model)
        except ValueError:
            if not self.warned:
                rospy.logwarn_throttle(5.0, "[gt_bridge] model '%s' not in /gazebo/model_states %s",
                                       self.model, msg.name)
            return
        self.pose = msg.pose[i]
        self.twist = msg.twist[i]

    def _make_odom(self, now):
        """One nav_msgs/Odometry: ENU pose + BODY-frame twist, child=base_link."""
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = self.frame
        od.child_frame_id = "base_link"
        od.pose.pose = self.pose
        if self.twist is not None:
            q = self.pose.orientation
            lx, ly, lz = _world_to_body(self.twist.linear.x, self.twist.linear.y,
                                        self.twist.linear.z, q)
            ax, ay, az = _world_to_body(self.twist.angular.x, self.twist.angular.y,
                                        self.twist.angular.z, q)
            od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = lx, ly, lz
            od.twist.twist.angular.x, od.twist.twist.angular.y, od.twist.twist.angular.z = ax, ay, az
        return od

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if self.pose is not None:
                now = rospy.Time.now()
                # ONE odometry (offset 0) -> EPIC + PX4, identical pose+vel.
                od = self._make_odom(now)
                self.odom_pub.publish(od)          # EPIC
                self.fcu_pub.publish(od)           # PX4 EKF2 external odometry

                if self.vis_pub is not None:       # optional pose-only fallback
                    ps = PoseStamped()
                    ps.header.stamp = now
                    ps.header.frame_id = self.frame
                    ps.pose = self.pose
                    self.vis_pub.publish(ps)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("gz_truth_bridge")
    GtBridge().spin()
