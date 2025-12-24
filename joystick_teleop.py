#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import pygame

class JoystickTeleop:
    def __init__(self):
        rospy.init_node('joystick_teleop')
        
        # Same speed parameters as keyboard teleop
        self.speed = 0.8
        self.turn = 1.0
        
        # Initialize pygame and joystick
        pygame.init()
        pygame.joystick.init()
        
        # Check for joysticks
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            rospy.logerr("No joysticks detected!")
            raise RuntimeError("No joysticks found")
        
        print(f"Found {joystick_count} joystick(s)")
        
        # Initialize the first joystick
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        # Get joystick info
        joystick_name = self.joystick.get_name()
        print(f"Joystick name: {joystick_name}")
        
        # Button states for toggle functionality
        self.holonomic_mode = False
        self.button1_pressed = False  # For toggle debouncing
        
        # Publisher
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # Display control info
        self.display_controls()
        
    def display_controls(self):
        """Display control instructions similar to keyboard teleop"""
        msg = """
        Joystick Teleop (Thrustmaster T.Flight Hotas X)
        ----------------------------------------------
        
        Default Mode (Non-Holonomic):
        - Axis 1 (Forward/Back): Forward/Backward movement
        - Axis 0 (Left/Right): Turn Left/Right
        - Axis 3 (Twist): Rotation Left/Right


        Holonomic Mode (Toggle with Button 1):
        - Axis 1 (Forward/Back): Forward/Backward
        - Axis 0 (Left/Right): Strafe Left/Right
        - Axis 3 (Twist): Rotation Left/Right
        
        Additional Controls:
        - Axis 2 (Up/Down): Vertical movement (linear.z)
        - Button 0 (Trigger): Hold for turbo (2x speed)
        - Button 1 (ThumbBtn): Toggle holonomic mode
        - Button 3 (TopBtn): Emergency stop and exit
        
        Speeds (same as keyboard):
        - Linear: {linear}
        - Angular: {angular}
        - Turbo: 2x speed
        
        CTRL-C or Button 3 to quit
        """.format(linear=self.speed, angular=self.turn)
        
        print(msg)
    
    def get_joystick_state(self):
        """Get current state of joystick axes and buttons"""
        pygame.event.pump()  # Process pygame events
        
        # Get axis values (apply deadzone)
        axes = []
        for i in range(min(self.joystick.get_numaxes(), 7)):  # Get up to 7 axes
            axis_value = self.joystick.get_axis(i)
            # Apply deadzone
            if abs(axis_value) < 0.1:
                axis_value = 0.0
            axes.append(axis_value)
        
        # Get button states
        buttons = []
        for i in range(min(self.joystick.get_numbuttons(), 12)):  # Get up to 12 buttons
            buttons.append(self.joystick.get_button(i))
        
        return axes, buttons
    
    def calculate_movement(self, axes, buttons):
        """Calculate twist message using keyboard teleop logic"""
        x = 0
        y = 0
        z = 0
        th = 0
        
        # Check for exit button (Button 3)
        # if buttons[3]:  # TopBtn
        #     return None
        
        # Toggle holonomic mode with Button 1 (with debounce)
        if buttons[1] and not self.button1_pressed:
            self.holonomic_mode = not self.holonomic_mode
            mode = "HOLONOMIC" if self.holonomic_mode else "NON-HOLONOMIC"
            print(f"Mode changed to: {mode}")
            self.button1_pressed = True
        elif not buttons[1]:
            self.button1_pressed = False
        
        # Check if turbo is pressed (Button 0)
        current_speed = self.speed * (2.0 if buttons[0] else 1.0)
        current_turn = self.turn * (2.0 if buttons[0] else 1.0)
        
        # Get axis values with proper mapping
        # Based on your testing: axes 0, 1, 2, 3 are the ones you want
        if len(axes) >= 4:
            axis0 = axes[0]  # Left/Right
            axis1 = axes[1]  # Forward/Back
            axis2 = axes[2]  # Up/Down
            axis3 = axes[3]  # Twist (rotation)
        else:
            axis0 = axis1 = axis2 = axis3 = 0.0
        
        # Apply threshold for movement (similar to key presses)
        threshold = 0.3
        
        if not self.holonomic_mode:
            # Non-holonomic mode (like WASD keys)
            if axis1 > threshold:  # Forward
                x = 1
            elif axis1 < -threshold:  # Backward
                x = -1

            if axis0 > threshold:  # Turn left (positive linear y)
                y = 1
            elif axis0 < -threshold:  # Turn right (negative linear y)
                y = -1

            # Rotation with twist axis (axis3)
            if axis3 > threshold:  # Rotate left (positive angular z)
                th = 1
            elif axis3 < -threshold:  # Rotate right (negative angular z)
                th = -1
                
        else:
            # Holonomic mode (like shift+WASD keys)
            # Forward/backward
            if axis1 > threshold:  # Forward
                x = 1
            elif axis1 < -threshold:  # Backward
                x = -1
            
            # Strafe left/right
            if axis0 > threshold:  # Strafe left (positive y)
                y = 1
            elif axis0 < -threshold:  # Strafe right (negative y)
                y = -1
            
            # Rotation with twist axis (axis3)
            if axis3 > threshold:  # Rotate left (positive angular z)
                th = 1
            elif axis3 < -threshold:  # Rotate right (negative angular z)
                th = -1
        
        # Up/Down movement (axis2)
        if axis2 > threshold:  # Down
            z = -1
        elif axis2 < -threshold:  # Up
            z = 1

        # Create twist message (same as keyboard teleop)
        twist = Twist()
        twist.linear.x = x * current_speed
        twist.linear.y = y * current_speed  # For holonomic movement
        twist.linear.z = z * current_speed  # Up/down movement
        
        # Fix axis3 inversion - invert the sign for intuitive control
        # If twist feels reversed, change the sign here:
        twist.angular.z = th * current_turn
        
        twist.angular.x = 0
        twist.angular.y = 0
        
        return twist
    
    def run(self):
        """Main loop - same update rate as keyboard teleop"""
        rate = rospy.Rate(10)  # Similar to keyboard polling rate
        
        print("Starting joystick teleop. Press Button 3 (TopBtn) or CTRL-C to exit.")
        
        try:
            while not rospy.is_shutdown():
                # Get joystick state
                axes, buttons = self.get_joystick_state()
                
                # Calculate movement
                twist = self.calculate_movement(axes, buttons)
                
                if twist is None:  # Exit requested
                    break
                
                # Publish twist message
                self.pub.publish(twist)
                
                # Display status occasionally
                if rospy.get_time() % 5 < 0.1:  # Every ~5 seconds
                    mode = "HOLONOMIC" if self.holonomic_mode else "NON-HOLONOMIC"
                    turbo = "ON" if buttons[0] else "OFF"
                    print(f"Mode: {mode}, Turbo: {turbo}, "
                          f"Linear: ({twist.linear.x:.2f}, {twist.linear.y:.2f}, {twist.linear.z:.2f}), "
                          f"Angular: {twist.angular.z:.2f}")
                
                rate.sleep()
                
        except pygame.error as e:
            rospy.logerr(f"Pygame error: {e}")
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources - same as keyboard teleop"""
        print("\nCleaning up...")
        
        # Stop the robot (same as keyboard teleop finally block)
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub.publish(twist)
        rospy.sleep(0.1)
        
        # Quit pygame
        pygame.quit()
        print("Joystick teleop shutdown complete.")

if __name__ == "__main__":
    try:
        teleop = JoystickTeleop()
        teleop.run()
    except Exception as e:
        rospy.logerr(f"Failed to initialize joystick teleop: {e}")
        pygame.quit()