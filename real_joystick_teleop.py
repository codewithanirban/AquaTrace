#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import pygame
import numpy as np

class JoystickTeleop:
    def __init__(self):
        rospy.init_node('joystick_teleop')
        
        # Same speed parameters as keyboard teleop
        self.max_speed = 0.8
        self.max_turn = 1.0
        
        # Inertia/damping parameters
        self.inertia_enabled = True
        self.damping_factor = 0.5  # How quickly speed reduces (0.1-0.9)
        self.angular_damping = 0.5  # Different damping for rotation

        # Current velocities (for inertia)
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_linear_z = 0.0
        self.current_angular_z = 0.0
        
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
        self.button5_pressed = False  # For inertia toggle
        
        # Publisher
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        
        # Display control info
        self.display_controls()
        
    def display_controls(self):
        """Display control instructions similar to keyboard teleop"""
        # msg = """
        # Joystick Teleop (Thrustmaster T.Flight Hotas X)
        # ----------------------------------------------
        
        # Default Mode (Non-Holonomic):
        # - Axis 1 (Forward/Back): Forward/Backward movement
        # - Axis 0 (Left/Right): Turn Left/Right
        # - Axis 3 (Twist): Rotation Left/Right

        # Holonomic Mode (Toggle with Button 1):
        # - Axis 1 (Forward/Back): Forward/Backward
        # - Axis 0 (Left/Right): Strafe Left/Right
        # - Axis 3 (Twist): Rotation Left/Right
        
        # Additional Controls:
        # - Axis 2 (Up/Down): Vertical movement (linear.z)
        # - Button 0 (Trigger): Hold for turbo (2x speed)
        # - Button 1 (ThumbBtn): Toggle holonomic mode
        # - Button 3 (TopBtn): Emergency stop and exit
        # - Button 5 (TopBtn2): Toggle inertia (current: {})
        
        # Speeds (same as keyboard):
        # - Linear: {linear}
        # - Angular: {angular}
        # - Turbo: 2x speed
        
        # Inertia Settings:
        # - Damping factor: {damping}
        # - Angular damping: {ang_damping}
        
        # CTRL-C or Button 3 to quit
        # """.format(
        #     linear=self.max_speed, 
        #     angular=self.max_turn,
        #     damping=self.damping_factor,
        #     ang_damping=self.angular_damping,
        #     inertia="ON" if self.inertia_enabled else "OFF"
        # )

        msg = f"""
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
        - Button 5 (PinkieBtn): Toggle inertia (current: {'ON' if self.inertia_enabled else 'OFF'})
        
        Speeds (same as keyboard):
        - Linear: {self.max_speed}
        - Angular: {self.max_turn}
        - Turbo: 2x speed
        
        Inertia Settings:
        - Damping factor: {self.damping_factor}
        - Angular damping: {self.angular_damping}
        
        CTRL-C or Button 3 to quit
        """
        
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
    
    def apply_inertia(self, target_linear_x, target_linear_y, target_linear_z, target_angular_z, dt):
        """
        Apply inertia/damping to make speed change gradually
        dt: time delta since last update (seconds)
        """
        if not self.inertia_enabled:
            # If inertia is disabled, use target values directly
            self.current_linear_x = target_linear_x
            self.current_linear_y = target_linear_y
            self.current_linear_z = target_linear_z
            self.current_angular_z = target_angular_z
            return
        
        # Calculate decay factor based on damping and time
        # Higher damping = slower decay
        linear_decay = 1.0 - (1.0 - self.damping_factor) * dt * 10
        angular_decay = 1.0 - (1.0 - self.angular_damping) * dt * 10
        
        # Clamp decay factors
        linear_decay = max(0.1, min(0.99, linear_decay))
        angular_decay = max(0.1, min(0.99, angular_decay))
        
        # Apply inertia to linear velocities
        # If joystick is providing input, move toward target
        # If not, gradually decay current velocity
        
        # X-axis linear movement
        if abs(target_linear_x) > 0.01:
            # Move toward target with some resistance
            diff = target_linear_x - self.current_linear_x
            self.current_linear_x += diff * (1.0 - self.damping_factor * 0.5) * dt * 10
        else:
            # Decay current velocity
            self.current_linear_x *= linear_decay
        
        # Y-axis linear movement
        if abs(target_linear_y) > 0.01:
            diff = target_linear_y - self.current_linear_y
            self.current_linear_y += diff * (1.0 - self.damping_factor * 0.5) * dt * 10
        else:
            self.current_linear_y *= linear_decay
        
        # Z-axis linear movement
        if abs(target_linear_z) > 0.01:
            diff = target_linear_z - self.current_linear_z
            self.current_linear_z += diff * (1.0 - self.damping_factor * 0.5) * dt * 10
        else:
            self.current_linear_z *= linear_decay
        
        # Angular movement
        if abs(target_angular_z) > 0.01:
            diff = target_angular_z - self.current_angular_z
            self.current_angular_z += diff * (1.0 - self.angular_damping * 0.5) * dt * 10
        else:
            self.current_angular_z *= angular_decay
        
        # Apply small deadzone to stop completely when very slow
        if abs(self.current_linear_x) < 0.01:
            self.current_linear_x = 0.0
        if abs(self.current_linear_y) < 0.01:
            self.current_linear_y = 0.0
        if abs(self.current_linear_z) < 0.01:
            self.current_linear_z = 0.0
        if abs(self.current_angular_z) < 0.01:
            self.current_angular_z = 0.0
    
    def calculate_movement(self, axes, buttons):
        """Calculate twist message using keyboard teleop logic"""
        target_x = 0
        target_y = 0
        target_z = 0
        target_th = 0
        
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

        # Toggle inertia with Button 5 (with debounce)
        if buttons[5] and not self.button5_pressed:
            self.inertia_enabled = not self.inertia_enabled
            state = "ON" if self.inertia_enabled else "OFF"
            print(f"Inertia mode: {state}")
            self.button5_pressed = True
        elif not buttons[5]:
            self.button5_pressed = False

        # Check if turbo is pressed (Button 0)
        current_speed = self.max_speed * (2.0 if buttons[0] else 1.0)
        current_turn = self.max_turn * (2.0 if buttons[0] else 1.0)
        
        # Get axis values with proper mapping
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
                target_x = -1
            elif axis1 < -threshold:  # Backward
                target_x = 1

            if axis0 > threshold:  # Turn left (positive linear y)
                target_y = -1
            elif axis0 < -threshold:  # Turn right (negative linear y)
                target_y = 1

            # Rotation with twist axis (axis3)
            if axis3 > threshold:  # Rotate left (positive angular z)
                target_th = -1
            elif axis3 < -threshold:  # Rotate right (negative angular z)
                target_th = 1

        else:
            # Holonomic mode (like shift+WASD keys)
            # Forward/backward
            if axis1 > threshold:  # Forward
                target_x = -1
            elif axis1 < -threshold:  # Backward
                target_x = 1

            # Strafe left/right
            if axis0 > threshold:  # Strafe left (positive y)
                target_y = -1
            elif axis0 < -threshold:  # Strafe right (negative y)
                target_y = 1
            
            # Rotation with twist axis (axis3)
            if axis3 > threshold:  # Rotate left (positive angular z)
                target_th = -1
            elif axis3 < -threshold:  # Rotate right (negative angular z)
                target_th = 1

        # Up/Down movement (axis2)
        if axis2 > threshold:  # Down
            target_z = -1
        elif axis2 < -threshold:  # Up
            target_z = 1

        # Scale target values
        target_linear_x = target_x * current_speed
        target_linear_y = target_y * current_speed
        target_linear_z = target_z * current_speed
        target_angular_z = target_th * current_turn
        
        return target_linear_x, target_linear_y, target_linear_z, target_angular_z
    
    def run(self):
        """Main loop - same update rate as keyboard teleop"""
        rate = rospy.Rate(20)  # Increased rate for smoother inertia
        last_time = rospy.Time.now().to_sec()
        
        print("Starting joystick teleop. Press Button 3 (TopBtn) or CTRL-C to exit.")
        
        status_counter = 0
        
        try:
            while not rospy.is_shutdown():
                # Calculate time delta for inertia
                current_time = rospy.Time.now().to_sec()
                dt = current_time - last_time
                last_time = current_time
                
                # Ensure dt is reasonable (avoid large jumps)
                if dt > 0.1:  # If dt is too large, cap it
                    dt = 0.05
                
                # Get joystick state
                axes, buttons = self.get_joystick_state()
                
                # Calculate target movement
                movement = self.calculate_movement(axes, buttons)
                
                if movement is None:  # Exit requested
                    break
                
                target_linear_x, target_linear_y, target_linear_z, target_angular_z = movement
                
                # Apply inertia/damping
                self.apply_inertia(
                    target_linear_x, target_linear_y, target_linear_z, target_angular_z, dt
                )
                
                # Create twist message with current (inertia-applied) velocities
                twist = Twist()
                twist.linear.x = self.current_linear_x
                twist.linear.y = self.current_linear_y
                twist.linear.z = self.current_linear_z
                twist.angular.z = self.current_angular_z
                twist.angular.x = 0
                twist.angular.y = 0
                
                # Publish twist message
                self.pub.publish(twist)
                
                # Display status occasionally (every ~2 seconds)
                status_counter += 1
                if status_counter >= 40:  # 40 iterations at 20Hz = 2 seconds
                    mode = "HOLONOMIC" if self.holonomic_mode else "NON-HOLONOMIC"
                    turbo = "ON" if buttons[0] else "OFF"
                    inertia = "ON" if self.inertia_enabled else "OFF"
                    
                    print(f"Mode: {mode}, Turbo: {turbo}, Inertia: {inertia}")
                    print(f"Target Linear: ({target_linear_x:.2f}, {target_linear_y:.2f}, {target_linear_z:.2f})")
                    print(f"Current Linear: ({self.current_linear_x:.2f}, {self.current_linear_y:.2f}, {self.current_linear_z:.2f})")
                    print(f"Angular: {self.current_angular_z:.2f}")
                    print("-" * 40)
                    
                    status_counter = 0
                
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
        
        # Gradually reduce speed to zero (for smooth stop)
        if self.inertia_enabled:
            print("Gradually stopping...")
            for i in range(10):
                # Apply damping to current velocities
                self.current_linear_x *= 0.5
                self.current_linear_y *= 0.5
                self.current_linear_z *= 0.5
                self.current_angular_z *= 0.5
                
                # Create and publish twist
                twist = Twist()
                twist.linear.x = self.current_linear_x
                twist.linear.y = self.current_linear_y
                twist.linear.z = self.current_linear_z
                twist.angular.z = self.current_angular_z
                self.pub.publish(twist)
                
                rospy.sleep(0.05)  # Small delay
        
        # Final stop (same as keyboard teleop finally block)
        twist = Twist()
        twist.linear.x = 0
        twist.linear_y = 0
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
        if pygame.get_init():
            pygame.quit()