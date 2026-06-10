#!/usr/bin/env python3
"""
px4_setpoint_adapter.py - HILS PX4 output stage: receive the onboard's PATH and
fly it on PX4 (Gazebo HITL).

The real onboard publishes its trajectory as trajectory_msgs/MultiDOFJointTrajectory
on /trajectory/multidof_trajectory (custom_trajectory_sampler, samples EPIC's MINCO
trajectory into pos+vel points). This adapter is HILS's "consumer": it samples that
trajectory by elapsed time and republishes the current setpoint as the standard PX4
type mavros_msgs/PositionTarget on /mavros/setpoint_raw/local, then drives the
OFFBOARD + ARM sequence. EPIC core is untouched - this is purely the PX4 output stage.

Interface (unified with the onboard, like the odom = nav_msgs/Odometry contract):
  PATH IN : trajectory_msgs/MultiDOFJointTrajectory on ~traj_topic
            (each point: transforms[0]=pose, velocities[0]=vel, time_from_start)
  SP  OUT : mavros_msgs/PositionTarget on /mavros/setpoint_raw/local (ENU->NED)

Also keeps a PositionCommand input (~input_topic) for HILS-internal hover/climb
(test_hover_cmd) before EPIC takes over; the most-recent MultiDOF trajectory wins.

Frames: ENU (EPIC world) in, MAVROS converts to PX4 NED. Origin/yaw aligned.

Params (~private):
  ~traj_topic    (str,  default /trajectory/multidof_trajectory)  EPIC path (MultiDOF)
  ~input_topic   (str,  default /position_cmd)   hover/climb PositionCommand (kept)
  ~rate          (float,default 50.0)  setpoint stream rate [Hz]
  ~auto_offboard (bool, default true)  auto switch to OFFBOARD once streaming
  ~auto_arm      (bool, default false) auto arm (SAFETY: off by default)
  ~use_vel       (bool, default false) feed-forward velocity (from the trajectory)
  ~traj_timeout  (float,default 1.5)   [s] MultiDOF considered stale after this
"""
import math
import rospy
from quadrotor_msgs.msg import PositionCommand
from trajectory_msgs.msg import MultiDOFJointTrajectory
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode, CommandBool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

PT = PositionTarget
NAN = float("nan")


def _nan_vec():
    return Vector3(NAN, NAN, NAN)


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SetpointAdapter:
    def __init__(self):
        self.traj_topic = rospy.get_param("~traj_topic", "/trajectory/multidof_trajectory")
        self.input_topic = rospy.get_param("~input_topic", "/position_cmd")
        self.rate_hz = float(rospy.get_param("~rate", 50.0))
        self.auto_offboard = bool(rospy.get_param("~auto_offboard", True))
        self.auto_arm = bool(rospy.get_param("~auto_arm", False))
        self.use_vel = bool(rospy.get_param("~use_vel", False))
        self.traj_timeout = float(rospy.get_param("~traj_timeout", 1.5))

        self.state = State()
        self.odom = None
        self.cmd = None              # latest PositionCommand (hover/climb)
        self.traj = None             # latest MultiDOFJointTrajectory points (EPIC path)
        self.traj_recv = rospy.Time(0)
        self.last_req = rospy.Time(0)

        self.sp_pub = rospy.Publisher("/mavros/setpoint_raw/local", PT, queue_size=10)
        rospy.Subscriber("/mavros/state", State, self._state_cb)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._odom_cb)
        rospy.Subscriber(self.traj_topic, MultiDOFJointTrajectory, self._traj_cb)
        rospy.Subscriber(self.input_topic, PositionCommand, self._cmd_cb)

        rospy.loginfo("[adapter] waiting for MAVROS services...")
        rospy.wait_for_service("/mavros/set_mode")
        rospy.wait_for_service("/mavros/cmd/arming")
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        rospy.loginfo("[adapter] ready. path=%s hover=%s rate=%.0fHz auto_offboard=%s auto_arm=%s use_vel=%s",
                      self.traj_topic, self.input_topic, self.rate_hz,
                      self.auto_offboard, self.auto_arm, self.use_vel)

    def _state_cb(self, msg):
        self.state = msg

    def _odom_cb(self, msg):
        self.odom = msg

    def _cmd_cb(self, msg):
        self.cmd = msg

    def _traj_cb(self, msg):
        if msg.points:
            self.traj = msg.points
            self.traj_recv = rospy.Time.now()

    def _traj_active(self):
        return (self.traj is not None and
                (rospy.Time.now() - self.traj_recv).to_sec() < self.traj_timeout)

    def _sample_traj(self):
        """Pick the trajectory point at elapsed time since the trajectory arrived.
        EPIC replans frequently (each new traj starts ~at the current pose), so
        elapsed-since-receive tracks the path forward and resets on every replan."""
        elapsed = (rospy.Time.now() - self.traj_recv).to_sec()
        pt = self.traj[-1]
        for p in self.traj:
            if p.time_from_start.to_sec() >= elapsed:
                pt = p
                break
        return pt

    def _build_target(self):
        m = PT()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.coordinate_frame = PT.FRAME_LOCAL_NED  # mavros: ENU in, NED out
        # Unused fields -> NaN (so PX4 doesn't read velocity=0 as "hold zero vel").
        ignore = PT.IGNORE_YAW_RATE
        if not self.use_vel:
            ignore |= PT.IGNORE_VX | PT.IGNORE_VY | PT.IGNORE_VZ
        ignore |= PT.IGNORE_AFX | PT.IGNORE_AFY | PT.IGNORE_AFZ  # sampler gives no accel
        m.type_mask = ignore
        m.yaw_rate = NAN

        if self._traj_active():
            # EPIC path (MultiDOFJointTrajectory) - the unified interface.
            pt = self._sample_traj()
            tf = pt.transforms[0]
            m.position.x = tf.translation.x
            m.position.y = tf.translation.y
            m.position.z = tf.translation.z
            m.yaw = _yaw_from_quat(tf.rotation)
            if self.use_vel and pt.velocities:
                m.velocity = pt.velocities[0].linear
            else:
                m.velocity = _nan_vec()
            m.acceleration_or_force = _nan_vec()
        elif self.cmd is not None:
            # HILS-internal hover/climb (PositionCommand) before EPIC takes over.
            c = self.cmd
            m.position = c.position
            m.velocity = c.velocity if self.use_vel else _nan_vec()
            m.acceleration_or_force = _nan_vec()
            m.yaw = c.yaw
        elif self.odom is not None:
            # no input yet: hold current position (required to enter OFFBOARD)
            p = self.odom.pose.pose.position
            m.position.x, m.position.y, m.position.z = p.x, p.y, p.z
            m.velocity = _nan_vec()
            m.acceleration_or_force = _nan_vec()
            m.yaw = 0.0
            m.type_mask = (PT.IGNORE_VX | PT.IGNORE_VY | PT.IGNORE_VZ |
                           PT.IGNORE_AFX | PT.IGNORE_AFY | PT.IGNORE_AFZ |
                           PT.IGNORE_YAW_RATE)
        else:
            return None
        return m

    def _maybe_offboard_arm(self):
        if (rospy.Time.now() - self.last_req) < rospy.Duration(2.0):
            return
        self.last_req = rospy.Time.now()
        if self.auto_offboard and self.state.mode != "OFFBOARD":
            try:
                ok = self.set_mode(custom_mode="OFFBOARD").mode_sent
                rospy.loginfo("[adapter] OFFBOARD request sent=%s (cur=%s)", ok, self.state.mode)
            except rospy.ServiceException as e:
                rospy.logwarn("[adapter] set_mode failed: %s", e)
        elif self.auto_arm and not self.state.armed:
            try:
                ok = self.arming(True).success
                rospy.loginfo("[adapter] ARM request success=%s", ok)
            except rospy.ServiceException as e:
                rospy.logwarn("[adapter] arming failed: %s", e)

    def spin(self):
        r = rospy.Rate(self.rate_hz)
        # pre-stream a few setpoints before any mode change (PX4 requirement)
        while not rospy.is_shutdown() and not self.state.connected:
            rospy.loginfo_throttle(2.0, "[adapter] waiting for FCU connection...")
            r.sleep()
        for _ in range(int(self.rate_hz)):  # ~1s of setpoints
            if rospy.is_shutdown():
                return
            t = self._build_target()
            if t is not None:
                self.sp_pub.publish(t)
            r.sleep()
        rospy.loginfo("[adapter] streaming started")
        while not rospy.is_shutdown():
            t = self._build_target()
            if t is not None:
                self.sp_pub.publish(t)
            self._maybe_offboard_arm()
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("px4_setpoint_adapter")
    SetpointAdapter().spin()
