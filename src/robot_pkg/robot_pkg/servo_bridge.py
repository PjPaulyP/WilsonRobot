import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit
import numpy as np
from robot_pkg import robot_config as robot_config_

class ServoBridge(Node):

    def __init__(self, robot_config):
        
        super().__init__('servo_bridge_node')
        
        self.robot_config = robot_config
        self.kit = ServoKit(channels=16) # servo channels must be 8 or 16
        for name, channels in self.robot_config.servo_map.items():
            self.kit.servo[channels].set_pulse_width_range(
                self.robot_config.servo_pulsewidth_range[name][0], # min 
                self.robot_config.servo_pulsewidth_range[name][1]  # max
            )
            self.kit.servo[channels].actuation_range = self.robot_config.servo_actuation_range[name]
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
         

    def listener_callback(self, msg):
        
        for joint_name, pos in zip(msg.name, msg.position):

            if joint_name not in self.robot_config.servo_map:
                self.get_logger().warn(f"Unknown joint name: {joint_name}")
                continue

            channel = self.robot_config.servo_map[joint_name]

            # Add 90deg to convert to 0-180deg servo angle
            angle_deg = 90 + np.degrees(pos) * self.robot_config.arm_rotation_multiplier[joint_name] * self.robot_config.actuation_axis_co_rotation_multiplier[joint_name]
            angle_deg += self.robot_config.joint_offsets[joint_name]

            print(f"Joint: {joint_name}, Angle: {angle_deg}")

            # Clamp to servo safe range
            angle_deg = self.clamp(angle_deg, self.robot_config.joint_limit_mins[joint_name], self.robot_config.joint_limit_maxs[joint_name])

            # Send to servo
            channel = self.robot_config.servo_map[joint_name]
            try:
                self.kit.servo[channel].angle = angle_deg
            except Exception as e:
                self.get_logger().error(f"Servo {joint_name} (ch {channel}) error: {e}")
        
    def clamp(self, val, min_val, max_val):
        return max(min(val, max_val), min_val)

def main(args=None):
    rclpy.init(args=args)
    robot_config = robot_config_.RobotConfig()
    node = ServoBridge(robot_config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()