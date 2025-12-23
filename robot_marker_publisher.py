
#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import math

class RobotMarkerPublisher:
    def __init__(self):
        rospy.init_node("robot_marker_publisher_node")

        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        rospy.Subscriber("robot_pose", PoseStamped, self.pose_callback)

        self.marker = Marker()
        self.marker.header.frame_id = "odom"  # The frame in which the marker is defined
        self.marker.ns = "robot_body"
        self.marker.id = 0
        self.marker.type = Marker.CUBE # A cube to represent the robot
        self.marker.action = Marker.ADD

        self.marker.scale.x = 0.5  # Robot length
        self.marker.scale.y = 0.3  # Robot width
        self.marker.scale.z = 0.2  # Robot height

        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0  # Blue color
        self.marker.color.a = 1.0  # Opaque

        self.marker.lifetime = rospy.Duration() # 0 means forever

    def pose_callback(self, msg):
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose = msg.pose
        self.marker_pub.publish(self.marker)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        rmp = RobotMarkerPublisher()
        rmp.run()
    except rospy.ROSInterruptException:
        pass
