#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import math

class DroneMarkerPublisher:
    def __init__(self):
        rospy.init_node("drone_marker_publisher_node")
        
        self.marker_pub = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
        rospy.Subscriber("robot_pose", PoseStamped, self.pose_callback)
        
        # Drone dimensions
        self.body_radius = 0.15
        self.body_height = 0.1
        self.arm_length = 0.3
        self.rotor_radius = 0.08
        self.rotor_thickness = 0.02
        
    def create_drone_markers(self, pose):
        """Create markers to represent a quadcopter drone"""
        markers = MarkerArray()
        
        # Main drone body (center)
        body = Marker()
        body.header.frame_id = "odom"
        body.header.stamp = rospy.Time.now()
        body.ns = "drone"
        body.id = 0
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose = pose.pose
        body.pose.position.z += self.body_height/2
        
        body.scale.x = self.body_radius * 2
        body.scale.y = self.body_radius * 2
        body.scale.z = self.body_height
        
        body.color.r = 0.3
        body.color.g = 0.3
        body.color.b = 0.3  # Dark gray
        body.color.a = 0.9
        
        body.lifetime = rospy.Duration()
        markers.markers.append(body)
        
        # Arms (4 arms in X configuration)
        arm_positions = [
            (self.arm_length/2, self.arm_length/2, 0),    # Front right
            (self.arm_length/2, -self.arm_length/2, 0),   # Front left
            (-self.arm_length/2, self.arm_length/2, 0),   # Back right
            (-self.arm_length/2, -self.arm_length/2, 0)   # Back left
        ]
        
        for i, (dx, dy, dz) in enumerate(arm_positions, start=1):
            arm = Marker()
            arm.header.frame_id = "odom"
            arm.header.stamp = rospy.Time.now()
            arm.ns = "drone"
            arm.id = i
            arm.type = Marker.CUBE
            arm.action = Marker.ADD
            
            # Position arm relative to body
            arm.pose.position.x = pose.pose.position.x + dx * math.cos(pose.pose.orientation.z) - dy * math.sin(pose.pose.orientation.z)
            arm.pose.position.y = pose.pose.position.y + dx * math.sin(pose.pose.orientation.z) + dy * math.cos(pose.pose.orientation.z)
            arm.pose.position.z = pose.pose.position.z
            arm.pose.orientation = pose.pose.orientation
            
            # Set arm dimensions (long and thin)
            arm_length = math.sqrt(dx*dx + dy*dy)
            arm.scale.x = arm_length
            arm.scale.y = 0.02
            arm.scale.z = 0.01
            
            arm.color.r = 0.6
            arm.color.g = 0.6
            arm.color.b = 0.6  # Light gray
            arm.color.a = 0.8
            
            arm.lifetime = rospy.Duration()
            markers.markers.append(arm)
        
        # Rotors (4 rotors at arm ends)
        for i, (dx, dy, dz) in enumerate(arm_positions, start=5):
            rotor = Marker()
            rotor.header.frame_id = "odom"
            rotor.header.stamp = rospy.Time.now()
            rotor.ns = "drone"
            rotor.id = i
            rotor.type = Marker.CYLINDER
            rotor.action = Marker.ADD
            
            # Position rotor at end of arm
            rotor.pose.position.x = pose.pose.position.x + dx * math.cos(pose.pose.orientation.z) - dy * math.sin(pose.pose.orientation.z)
            rotor.pose.position.y = pose.pose.position.y + dx * math.sin(pose.pose.orientation.z) + dy * math.cos(pose.pose.orientation.z)
            rotor.pose.position.z = pose.pose.position.z + 0.02
            rotor.pose.orientation = pose.pose.orientation
            
            rotor.scale.x = self.rotor_radius * 2
            rotor.scale.y = self.rotor_radius * 2
            rotor.scale.z = self.rotor_thickness
            
            # Alternate rotor colors for visualization
            if i % 2 == 0:
                rotor.color.r = 0.9
                rotor.color.g = 0.1
                rotor.color.b = 0.1  # Red
            else:
                rotor.color.r = 0.1
                rotor.color.g = 0.1
                rotor.color.b = 0.9  # Blue
            rotor.color.a = 0.7
            
            rotor.lifetime = rospy.Duration()
            markers.markers.append(rotor)
        
        # Direction indicator (optional)
        indicator = Marker()
        indicator.header.frame_id = "odom"
        indicator.header.stamp = rospy.Time.now()
        indicator.ns = "drone"
        indicator.id = 9
        indicator.type = Marker.SPHERE
        indicator.action = Marker.ADD
        
        indicator.pose = pose.pose
        indicator.pose.position.x += 0.2 * math.cos(pose.pose.orientation.z)
        indicator.pose.position.y += 0.2 * math.sin(pose.pose.orientation.z)
        indicator.pose.position.z += 0.05
        
        indicator.scale.x = 0.03
        indicator.scale.y = 0.03
        indicator.scale.z = 0.03
        
        indicator.color.r = 0.0
        indicator.color.g = 1.0
        indicator.color.b = 0.0  # Green
        indicator.color.a = 1.0
        
        indicator.lifetime = rospy.Duration()
        markers.markers.append(indicator)
        
        return markers
    
    def pose_callback(self, msg):
        markers = self.create_drone_markers(msg)
        self.marker_pub.publish(markers)

if __name__ == "__main__":
    try:
        publisher = DroneMarkerPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass