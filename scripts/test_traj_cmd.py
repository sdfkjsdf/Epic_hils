#!/usr/bin/env python3
"""
test_traj_cmd.py - Publish a MOVING /position_cmd trajectory and measure tracking.

This is the path-tracking validation harness (HILS goal step 2). It publishes a
known, repeatable quadrotor_msgs/PositionCommand trajectory on /position_cmd -
exactly the topic EPIC drives - so it exercises the same adapter -> PX4 -> Gazebo
path as a real EPIC trajectory, but with a controllable, analytic reference whose
position/velocity/acceleration are known exactly. It then prints the tracking
error (commanded vs actual odom = Gazebo truth in gazebo_truth mode), which is the
metric to tune PX4 gains against.

The PositionCommand always carries pos+vel+acc+yaw. Whether PX4 uses the vel/acc
feed-forward depends on the adapter's ~use_vel/~use_accel (turn them on for tight
tracking once position-only basics pass).

Phases: (1) climb to ~alt above the start point and settle, (2) fly the shape for
~laps, (3) hold the final point. Land/disarm is done by the caller (menu).

Params:
  ~shape   (str,   circle|figure8|square|line ; default circle)
  ~radius  (float, 2.0)  [m]   shape size (half-side for square, half-length for line)
  ~alt     (float, 1.5)  [m]   flight altitude (absolute)
  ~period  (float, 20.0) [s]   time for one lap / cycle
  ~laps    (float, 2.0)        number of laps
  ~yaw_mode(str,   fixed|forward ; default fixed)
"""
import math
import rospy
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Traj:
    def __init__(self):
        self.shape = rospy.get_param("~shape", "circle")
        self.R = float(rospy.get_param("~radius", 2.0))
        self.alt = float(rospy.get_param("~alt", 1.5))
        self.period = float(rospy.get_param("~period", 20.0))
        self.laps = float(rospy.get_param("~laps", 2.0))
        self.yaw_mode = rospy.get_param("~yaw_mode", "fixed")
        self.odom = None
        self.pub = rospy.Publisher("/position_cmd", PositionCommand, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._cb)
        rospy.loginfo("[traj] waiting for odom ...")
        while not rospy.is_shutdown() and self.odom is None:
            rospy.sleep(0.1)
        o = self.odom.pose.pose
        self.x0, self.y0 = o.position.x, o.position.y
        self.yaw0 = yaw_of(o.orientation)
        rospy.loginfo("[traj] shape=%s R=%.1f alt=%.1f period=%.1fs laps=%.1f start=(%.2f,%.2f)",
                      self.shape, self.R, self.alt, self.period, self.laps, self.x0, self.y0)

    def _cb(self, m):
        self.odom = m

    def ref(self, t):
        """Return (px,py,pz, vx,vy,vz, ax,ay,az) for trajectory time t.
        Centered so that t=0 starts at the climb point (x0,y0)."""
        w = 2 * math.pi / self.period
        R = self.R
        s = self.shape
        pz, vz, az = self.alt, 0.0, 0.0
        if s == "circle":
            # center to the -x so phase 0 is at the start point
            cx, cy = self.x0 - R, self.y0
            u = w * t
            px, py = cx + R * math.cos(u), cy + R * math.sin(u)
            vx, vy = -R * w * math.sin(u), R * w * math.cos(u)
            ax, ay = -R * w * w * math.cos(u), -R * w * w * math.sin(u)
        elif s == "figure8":
            # lemniscate-ish: x ~ sin(u), y ~ sin(2u)
            u = w * t
            px = self.x0 + R * math.sin(u)
            py = self.y0 + R * math.sin(2 * u)
            vx = R * w * math.cos(u)
            vy = R * 2 * w * math.cos(2 * u)
            ax = -R * w * w * math.sin(u)
            ay = -R * 4 * w * w * math.sin(2 * u)
        elif s == "line":
            # back-and-forth along x: x ~ sin(u)
            u = w * t
            px = self.x0 + R * math.sin(u)
            py = self.y0
            vx = R * w * math.cos(u)
            vy = 0.0
            ax = -R * w * w * math.sin(u)
            ay = 0.0
        else:  # square (constant-speed perimeter, 4 legs per period)
            perim = 8.0 * R
            speed = perim / self.period
            d = (speed * t) % perim
            leg = d / (2 * R)            # 0..4
            f = d - int(leg) * 2 * R     # 0..2R along current leg
            corners = [(-R, -R), (R, -R), (R, R), (-R, R)]
            dirs = [(1, 0), (0, 1), (-1, 0), (0, -1)]
            i = int(leg) % 4
            cx0, cy0 = corners[i]
            dx, dy = dirs[i]
            px = self.x0 + cx0 + dx * f
            py = self.y0 + cy0 + dy * f
            vx, vy = dx * speed, dy * speed
            ax, ay = 0.0, 0.0
        return px, py, pz, vx, vy, vz, ax, ay, az

    def spin(self):
        rate = rospy.Rate(50)
        # phase 1: climb to alt at start, settle
        rospy.loginfo("[traj] climbing to %.1f m ...", self.alt)
        t_climb = rospy.Time.now()
        while not rospy.is_shutdown():
            self._publish(self.x0, self.y0, self.alt, 0, 0, 0, 0, 0, 0, self.yaw0)
            z = self.odom.pose.pose.position.z
            if abs(z - self.alt) < 0.15 and (rospy.Time.now() - t_climb).to_sec() > 4:
                break
            if (rospy.Time.now() - t_climb).to_sec() > 25:
                rospy.logwarn("[traj] climb timeout (z=%.2f); continuing", z)
                break
            rate.sleep()
        rospy.sleep(1.0)
        # phase 2: fly the shape, report tracking error
        rospy.loginfo("[traj] flying %s for %.1f laps ...", self.shape, self.laps)
        t0 = rospy.Time.now()
        T = self.laps * self.period
        n, se = 0, 0.0
        emax = 0.0
        last_report = 0.0
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - t0).to_sec()
            if t > T:
                break
            px, py, pz, vx, vy, vz, ax, ay, az = self.ref(t)
            if self.yaw_mode == "forward" and (vx * vx + vy * vy) > 1e-3:
                yaw = math.atan2(vy, vx)
            else:
                yaw = self.yaw0
            self._publish(px, py, pz, vx, vy, vz, ax, ay, az, yaw)
            # tracking error vs actual (odom = Gazebo truth in gazebo_truth mode)
            a = self.odom.pose.pose.position
            e = math.sqrt((a.x - px) ** 2 + (a.y - py) ** 2 + (a.z - pz) ** 2)
            n += 1
            se += e * e
            emax = max(emax, e)
            if t - last_report >= 2.0:
                last_report = t
                rospy.loginfo("[traj] t=%4.1fs cmd=(%.2f,%.2f,%.2f) act=(%.2f,%.2f,%.2f) err=%.2fm rms=%.2f max=%.2f",
                              t, px, py, pz, a.x, a.y, a.z, e, math.sqrt(se / n), emax)
            rate.sleep()
        rms = math.sqrt(se / n) if n else float("nan")
        rospy.loginfo("[traj] DONE: tracking RMS=%.3f m, max=%.3f m over %.0f samples", rms, emax, n)
        # phase 3: hold final point
        px, py, pz, *_ = self.ref(T)
        t_hold = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - t_hold).to_sec() < 3:
            self._publish(px, py, pz, 0, 0, 0, 0, 0, 0, self.yaw0)
            rate.sleep()

    def _publish(self, px, py, pz, vx, vy, vz, ax, ay, az, yaw):
        m = PositionCommand()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.position.x, m.position.y, m.position.z = px, py, pz
        m.velocity.x, m.velocity.y, m.velocity.z = vx, vy, vz
        m.acceleration.x, m.acceleration.y, m.acceleration.z = ax, ay, az
        m.yaw = yaw
        self.pub.publish(m)


if __name__ == "__main__":
    rospy.init_node("test_traj_cmd")
    Traj().spin()
