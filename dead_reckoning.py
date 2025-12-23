
#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

class DeadReckoning:
    def __init__(self):
        rospy.init_node("dead_reckoning_node")

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.pose_pub = rospy.Publisher("robot_pose", PoseStamped, queue_size=10)
        rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

        self.x = 0.0
        self.y = 0.0
        #for depth
        self.z = 0.0
        self.theta = 0.0

        self.last_time = rospy.Time.now()

    def cmd_vel_callback(self, msg):
        dt = (rospy.Time.now() - self.last_time).to_sec()
        self.last_time = rospy.Time.now()

        # Update pose based on linear and angular velocities
        linear_velocity_x = msg.linear.x
        linear_velocity_y = msg.linear.y
        linear_velocity_z = msg.linear.z
        angular_velocity = msg.angular.z

        # Simple integration for dead reckoning (including lateral/y velocity)
        # x_dot = v_x * cos(theta) - v_y * sin(theta)
        # y_dot = v_x * sin(theta) + v_y * cos(theta)
        self.x += (linear_velocity_x * math.cos(self.theta) - linear_velocity_y * math.sin(self.theta)) * dt
        self.y += (linear_velocity_x * math.sin(self.theta) + linear_velocity_y * math.cos(self.theta)) * dt
        self.z += linear_velocity_z * dt
        self.theta += angular_velocity * dt

        # Normalize theta to be within -pi to pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.publish_odometry(msg)

    def publish_odometry(self,msg):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z

        # Convert yaw to quaternion
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Set velocities
        odom.twist.twist.linear.x = msg.linear.x
        odom.twist.twist.linear.y = msg.linear.y
        odom.twist.twist.linear.z = msg.linear.z
        odom.twist.twist.angular.z = msg.angular.z

        self.odom_pub.publish(odom)

        # Also publish as PoseStamped for simpler visualization in RViz
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "odom"
        pose_stamped.pose.position.x = self.x
        pose_stamped.pose.position.y = self.y
        pose_stamped.pose.position.z = self.z
        pose_stamped.pose.orientation = odom.pose.pose.orientation
        self.pose_pub.publish(pose_stamped)

        # Publish Transform (odom -> base_link)
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0),
            (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w),
            rospy.Time.now(),
            "base_link",
            "odom"
        )

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        dr = DeadReckoning()
        dr.run()
    except rospy.ROSInterruptException:
        pass
