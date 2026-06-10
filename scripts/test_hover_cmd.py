#!/usr/bin/env python3
"""
test_hover_cmd.py - Publish a hover /position_cmd for bring-up testing.

Reads the current odom once, then continuously publishes a quadrotor_msgs/
PositionCommand at (current x, current y, target_z) with the current yaw. The
onboard adapter relays it to PX4 as a position setpoint, so an armed vehicle
climbs/descends to that altitude and holds (hover).

Params:
  ~climb (float, default 1.5) [m]  relative climb from current z (used if ~abs_z unset)
  ~abs_z (float, default NaN) [m]  absolute target altitude (overrides ~climb)
"""
import math
import rospy
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry


class Hover:
    def __init__(self):
        self.climb = float(rospy.get_param("~climb", 1.5))
        self.abs_z = float(rospy.get_param("~abs_z", float("nan")))
        self.odom = None
        self.pub = rospy.Publisher("/position_cmd", PositionCommand, queue_size=10)
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._cb)
        rospy.loginfo("[hover] waiting for odom...")
        while not rospy.is_shutdown() and self.odom is None:
            rospy.sleep(0.1)
        o = self.odom.pose.pose
        q = o.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        tz = self.abs_z if self.abs_z == self.abs_z else (o.position.z + self.climb)  # NaN check
        self.target = (o.position.x, o.position.y, tz)
        rospy.loginfo("[hover] start=(%.2f %.2f %.2f) target=(%.2f %.2f %.2f) yaw=%.2f",
                      o.position.x, o.position.y, o.position.z, *self.target, self.yaw)

    def _cb(self, m):
        self.odom = m

    def spin(self):
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            m = PositionCommand()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "map"
            m.position.x, m.position.y, m.position.z = self.target
            m.yaw = self.yaw
            self.pub.publish(m)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("test_hover_cmd")
    Hover().spin()
