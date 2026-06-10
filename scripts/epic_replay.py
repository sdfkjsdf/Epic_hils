#!/usr/bin/env python3
"""
epic_replay.py - Replay a recorded EPIC trajectory onto the HITL drone (route A).

Reads a rosbag of EPIC's position command (quadrotor_msgs/PositionCommand, recorded
from /planning/pos_cmd in EPIC's native MARSIM demo) and republishes it on
/position_cmd - the topic the onboard adapter feeds to PX4. The recorded poses are
in EPIC's garage-world frame (far from the HITL origin), so we TRANSLATE the whole
trajectory so its first sample maps to the HITL drone's current (x,y) at altitude
~alt, preserving the trajectory shape (incl. z and yaw variations). Velocity and
acceleration are translation-invariant and forwarded unchanged.

Phases: (1) climb to alt at the current point, (2) replay the EPIC trajectory at
its recorded timing (optionally time-scaled), printing tracking error (cmd vs odom
= Gazebo truth in gazebo_truth mode), (3) hold the final point. Land/disarm = caller.

Params:
  ~bag        (str, /root/poongsan_hils/epic_traj.bag)
  ~bag_topic  (str, /planning/pos_cmd)   topic name inside the bag
  ~alt        (float, 1.5) [m]           altitude the trajectory start maps to
  ~speed      (float, 1.0)               time scale (0.5 = half speed / gentler)
  ~max_dur    (float, 0.0) [s]           cap replay duration (0 = full bag)
"""
import math
import sys
import rospy
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry

try:
    import rosbag
except ImportError:
    sys.stderr.write("rosbag python not available\n")
    sys.exit(1)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Replay:
    def __init__(self):
        self.bagp = rospy.get_param("~bag", "/root/poongsan_hils/epic_traj.bag")
        self.btopic = rospy.get_param("~bag_topic", "/planning/pos_cmd")
        self.alt = float(rospy.get_param("~alt", 1.5))
        self.speed = float(rospy.get_param("~speed", 1.0))
        self.max_dur = float(rospy.get_param("~max_dur", 0.0))
        self.odom = None
        self.pub = rospy.Publisher("/position_cmd", PositionCommand, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._cb)

        # load the EPIC trajectory from the bag: list of (t_rel, PositionCommand)
        self.cmds = []
        t0 = None
        bag = rosbag.Bag(self.bagp)
        for _, msg, t in bag.read_messages(topics=[self.btopic]):
            if t0 is None:
                t0 = t.to_sec()
            self.cmds.append((t.to_sec() - t0, msg))
        bag.close()
        if not self.cmds:
            rospy.logerr("[replay] no %s messages in %s", self.btopic, self.bagp)
            sys.exit(2)
        rospy.loginfo("[replay] loaded %d cmds (%.1fs) from %s",
                      len(self.cmds), self.cmds[-1][0], self.bagp)

    def _cb(self, m):
        self.odom = m

    def _pub(self, x, y, z, vx, vy, vz, ax, ay, az, yaw):
        m = PositionCommand()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.position.x, m.position.y, m.position.z = x, y, z
        m.velocity.x, m.velocity.y, m.velocity.z = vx, vy, vz
        m.acceleration.x, m.acceleration.y, m.acceleration.z = ax, ay, az
        m.yaw = yaw
        self.pub.publish(m)

    def spin(self):
        rospy.loginfo("[replay] waiting for odom ...")
        while not rospy.is_shutdown() and self.odom is None:
            rospy.sleep(0.1)
        o = self.odom.pose.pose
        cx, cy = o.position.x, o.position.y
        yaw0 = yaw_of(o.orientation)
        # offset: EPIC first sample -> (cx, cy, alt)
        p0 = self.cmds[0][1].position
        ox, oy, oz = cx - p0.x, cy - p0.y, self.alt - p0.z
        rospy.loginfo("[replay] offset=(%.1f,%.1f,%.1f) start_map=(%.2f,%.2f,%.2f)",
                      ox, oy, oz, cx, cy, self.alt)

        rate = rospy.Rate(50)
        # phase 1: climb to alt at current point
        rospy.loginfo("[replay] climbing to %.1f m ...", self.alt)
        tclimb = rospy.Time.now()
        while not rospy.is_shutdown():
            self._pub(cx, cy, self.alt, 0, 0, 0, 0, 0, 0, yaw0)
            if abs(self.odom.pose.pose.position.z - self.alt) < 0.15 and \
               (rospy.Time.now() - tclimb).to_sec() > 4:
                break
            if (rospy.Time.now() - tclimb).to_sec() > 25:
                break
            rate.sleep()
        rospy.sleep(1.0)

        # phase 2: replay at recorded timing (scaled), report tracking error
        rospy.loginfo("[replay] replaying EPIC trajectory (speed=%.2f) ...", self.speed)
        t_start = rospy.Time.now()
        n, se, emax, last = 0, 0.0, 0.0, 0.0
        for trel, c in self.cmds:
            if rospy.is_shutdown():
                break
            target = trel / max(self.speed, 1e-3)
            if self.max_dur > 0 and target > self.max_dur:
                break
            # pace to the (scaled) recorded time
            while not rospy.is_shutdown() and (rospy.Time.now() - t_start).to_sec() < target:
                rospy.sleep(0.002)
            x, y, z = c.position.x + ox, c.position.y + oy, c.position.z + oz
            self._pub(x, y, z, c.velocity.x, c.velocity.y, c.velocity.z,
                      c.acceleration.x, c.acceleration.y, c.acceleration.z, c.yaw)
            a = self.odom.pose.pose.position
            e = math.sqrt((a.x - x) ** 2 + (a.y - y) ** 2 + (a.z - z) ** 2)
            n += 1
            se += e * e
            emax = max(emax, e)
            now = (rospy.Time.now() - t_start).to_sec()
            if now - last >= 2.0:
                last = now
                rospy.loginfo("[replay] t=%4.1fs cmd=(%.1f,%.1f,%.1f) act=(%.1f,%.1f,%.1f) err=%.2f rms=%.2f max=%.2f",
                              now, x, y, z, a.x, a.y, a.z, e, math.sqrt(se / n), emax)
        rms = math.sqrt(se / n) if n else float("nan")
        rospy.loginfo("[replay] DONE: EPIC-trajectory tracking RMS=%.3f m, max=%.3f m (%d samples)", rms, emax, n)
        # phase 3: hold last point
        last_c = self.cmds[-1][1].position
        hx, hy, hz = last_c.x + ox, last_c.y + oy, last_c.z + oz
        th = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - th).to_sec() < 3:
            self._pub(hx, hy, hz, 0, 0, 0, 0, 0, 0, yaw0)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("epic_replay")
    Replay().spin()
