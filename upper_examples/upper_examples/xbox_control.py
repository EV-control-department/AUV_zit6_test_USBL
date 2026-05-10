#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from zit6_interfaces.msg import ZitSetpoint
from std_msgs.msg import UInt32
import threading
import time
import os

# Set SDL to use dummy video driver (since we only need joystick)
os.environ["SDL_VIDEODRIVER"] = "dummy"

try:
    import pygame
except ImportError:
    print("Error: 'pygame' library not found. Please install with: sudo apt install python3-pygame")
    pygame = None

class XboxControlNode(Node):
    def __init__(self):
        super().__init__('xbox_control_node')
        self.setpoint_pub = self.create_publisher(ZitSetpoint, '/zit6/cmd/setpoint', 10)
        self.hbt_pub = self.create_publisher(UInt32, '/zit6/cmd/agxhbt', 10)
        
        self.active = False
        self.axes = [0.0, 0.0, 0.0, 0.0] # Surge, Sway, Heave, Yaw
        self.last_a_state = False
        
        # Initialize Pygame Joystick
        if pygame:
            pygame.init()
            pygame.joystick.init()
            self.joystick_count = pygame.joystick.get_count()
            if self.joystick_count > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info(f"Found Joystick: {self.joystick.get_name()}")
            else:
                self.get_logger().error("No joystick found! Please connect an Xbox controller.")
                self.joystick = None
        
        # Start heartbeat and publishing loop
        self.create_timer(0.1, self.timer_callback)
        
        # Start input thread
        if pygame and self.joystick:
            self.input_thread = threading.Thread(target=self.read_joystick, daemon=True)
            self.input_thread.start()
        
        self.get_logger().info("Xbox Control Node Started. Press 'A' to toggle control.")

    def read_joystick(self):
        while rclpy.ok():
            pygame.event.pump()
            
            # Left Stick: Surge (Y), Sway (X)
            # According to user feedback, axis(1) is POSITIVE for Forward
            self.axes[0] = self.joystick.get_axis(1)  # Surge (Forward)
            self.axes[1] = -self.joystick.get_axis(0)  # Sway (Right)
            
            # Right Stick: Heave (Y), Yaw (X)
            self.axes[2] = self.joystick.get_axis(4) # Heave (Up is negative)
            self.axes[3] = self.joystick.get_axis(3)  # Yaw (Right is positive)
            
            # A Button
            a_state = self.joystick.get_button(0)
            if a_state and not self.last_a_state:
                self.active = not self.active
                self.get_logger().info(f"Control {'ACTIVATED' if self.active else 'DEACTIVATED'}")
                if not self.active:
                    self.send_stop_command()
            self.last_a_state = a_state
            
            time.sleep(0.02)

    def timer_callback(self):
        
        if self.active:
            msg = ZitSetpoint()
            msg.control_key = 50 # Force + Body + Inc (Wait, 50 is 0x32)
            msg.x = -self.apply_deadzone(self.axes[0])
            msg.y = self.apply_deadzone(self.axes[1])
            msg.z = self.apply_deadzone(self.axes[2])
            msg.yaw = self.apply_deadzone(self.axes[3])
            self.setpoint_pub.publish(msg)

    def apply_deadzone(self, val, deadzone=0.1):
        if abs(val) < deadzone:
            return 0.0
        return val

    def send_stop_command(self):
        # Switch to POS mode (0) to trigger stationary hold at current position
        msg = ZitSetpoint()
        msg.control_key = 16
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0
        msg.yaw = 0.0
        self.setpoint_pub.publish(msg)
        self.get_logger().info("Sent Stationary Hold Command (Mode Switch to 0)")

def main():
    rclpy.init()
    node = XboxControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
