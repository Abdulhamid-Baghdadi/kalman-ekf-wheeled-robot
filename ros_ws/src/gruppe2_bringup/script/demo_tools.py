#!/usr/bin/env python3
"""Demo helper for demo_kf_vs_ekf.launch (added for the portfolio recording, not part of the 2021 course work).

* drives the ROSbot around the table in the start room (waypoint follower on Gazebo's ground-truth pose,
  standing in for a human with keyboard teleop; the filters only see /rosbot/cmd_vel and noisy odometry)
* adds Gaussian noise (std 0.1 m / 0.1 rad, i.e. Q = 0.01 I as assumed by the filters)
  to Gazebo's odometry, which is otherwise almost perfect, and republishes it on /rosbot/odom_noisy
* publishes nav_msgs/Path trails for ground truth, noisy odometry, KF and EKF for RViz
"""
import math
import random

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler

NOISE = 0.1
# loop around the table in the start room of house.world (robot spawns at (-2, 1) facing +x)
WAYPOINTS = [(-1.0, 1.2), (-1.0, 3.8), (-3.9, 3.8), (-3.9, 1.2), (-2.0, 1.1)]


class Demo:
    def __init__(self):
        self.cmd = rospy.Publisher("/rosbot/cmd_vel", Twist, queue_size=1)
        self.noisy = rospy.Publisher("/rosbot/odom_noisy", Odometry, queue_size=1)
        self.paths = {k: rospy.Publisher(f"/demo/path_{k}", Path, queue_size=1, latch=True)
                      for k in ("truth", "odom", "kf", "ekf")}
        self.msgs = {k: Path() for k in self.paths}
        for p in self.msgs.values():
            p.header.frame_id = "odom"
        self.n = 0
        self.pose = None
        rospy.Subscriber("/rosbot/odom", Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber("/KF_Position", PoseWithCovarianceStamped, lambda m: self.add("kf", m.pose.pose), queue_size=1)
        rospy.Subscriber("/EKF_Position", PoseWithCovarianceStamped, lambda m: self.add("ekf", m.pose.pose), queue_size=1)

    def add(self, key, pose, every=5):
        if key in ("kf", "ekf") and self.n % every:
            return
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.header.stamp = rospy.Time.now()
        ps.pose = pose
        self.msgs[key].poses.append(ps)
        self.msgs[key].header.stamp = ps.header.stamp
        if len(self.msgs[key].poses) % 5 == 0:
            self.paths[key].publish(self.msgs[key])

    def on_odom(self, m):
        self.n += 1
        q = m.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
        out = Odometry()
        out.header, out.child_frame_id, out.twist = m.header, m.child_frame_id, m.twist
        out.pose.pose.position.x = m.pose.pose.position.x + random.gauss(0, NOISE)
        out.pose.pose.position.y = m.pose.pose.position.y + random.gauss(0, NOISE)
        qn = quaternion_from_euler(0, 0, yaw + random.gauss(0, NOISE))
        o = out.pose.pose.orientation
        o.x, o.y, o.z, o.w = qn
        self.noisy.publish(out)
        if self.n % 5 == 0:
            self.add("truth", m.pose.pose)
        if self.n % 10 == 0:
            self.add("odom", out.pose.pose)

    def drive(self):
        rate = rospy.Rate(20)
        for wx, wy in WAYPOINTS:
            while not rospy.is_shutdown():
                x, y, yaw = self.pose
                dist = math.hypot(wx - x, wy - y)
                if dist < 0.25:
                    break
                err = math.atan2(math.sin(math.atan2(wy - y, wx - x) - yaw), math.cos(math.atan2(wy - y, wx - x) - yaw))
                t = Twist()
                t.angular.z = max(-0.8, min(0.8, 2.0 * err))
                t.linear.x = 0.3 * max(0.0, math.cos(err)) ** 4
                self.cmd.publish(t)
                rate.sleep()
        for _ in range(20):
            self.cmd.publish(Twist())
            rate.sleep()
        rospy.loginfo("demo loop finished")


if __name__ == "__main__":
    rospy.init_node("demo_tools")
    demo = Demo()
    # start driving once /demo/start is set (the recording script sets it), or after ~start_delay seconds
    deadline = rospy.Time.now() + rospy.Duration(rospy.get_param("~start_delay", 5.0))
    while not rospy.is_shutdown() and not rospy.get_param("/demo/start", False) and rospy.Time.now() < deadline:
        rospy.sleep(0.2)
    while not rospy.is_shutdown() and demo.pose is None:
        rospy.sleep(0.1)
    demo.drive()
    rospy.spin()
