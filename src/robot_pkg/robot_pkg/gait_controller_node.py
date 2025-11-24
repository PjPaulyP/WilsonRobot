import numpy as np
import pandas as pd
from robot_pkg import kinematics as kin
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import JointState

from robot_pkg import robot_config as robot_config_
from robot_pkg import walk as walk_


class GaitController(Node):

    def __init__(self, robot_config, walk):
        super().__init__("gait_controller_node")
        self.robot = robot_config
        self.in_walk = False
        self.walk_start_time = None
    
        # walk object/class
        self.walk = walk
        
        # publisher: sensor_msgs/JointState
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # mode topic subscriber
        self.subscription = self.create_subscription(String, '/mode', self.mode_receiver, 10)

        # control timer
        self.control_period = 1 / float(robot_config.control_hz)
        self.timer = self.create_timer(self.control_period, self.control_tick)
    

    def vertical_wrt_leg_base(self, foot_height):
        """
        Calculate vertical displacement of foot with respect to leg base frame.

        Base frame is located at hip joint.

        foot_height: vertical displacement of foot with respect to ground
        neutral_stance_height: height of body frame with respect to ground in neutral stance

        Returns: vertical position of foot with respect to leg base frame 
        """
        # Global frame is positive upwards from ground so make negative here. 
        # Later will multiply by rotation matrix if base frame is tilted.
        return self.robot.neutral_stance_height - foot_height 

    def sagital_wrt_leg_base(self, foot_forward):
        """
        Calculate sagittal position of foot with respect to base frame. (forward/backward)

        Base frame is located at hip joint.

        foot_forward: sagittal position of foot with respect to body center
        neutral_stance_forward: distance from foot to hip in sagittal direction. Typically 0.

        Returns: sagittal position of foot with respect to base frame
        """
        return self.robot.neutral_stance_forward + foot_forward

    def transverse_wrt_leg_base(self, foot_transverse):
        """
        Calculate transverse displacement of foot with respect to base frame. (left/right)

        Base frame is located at hip joint.

        foot_transverse: transverse displacement of foot with respect to body center
        neutral_stance_transverse: distance from foot to hip in transverse direction. Typically the hip length.

        Returns: transverse displacement of foot with respect to base frame
        """
        return self.robot.neutral_stance_transverse + foot_transverse

    def target_foot_position_wrt_base(
        self,
        foot_height,
        foot_forward,
        foot_transverse,
    ):
        """
        Calculate foot position with respect to leg base frame.

        Base frame is located at hip joint.

        foot_height: vertical position of foot with respect to ground
        foot_forward: sagittal position of foot with respect to body center
        foot_transverse: transverse position of foot with respect to body center
        neutral_stance_height: height of body frame with respect to ground in neutral stance
        neutral_stance_forward: distance from foot to hip in sagittal direction. Typically 0.
        neutral_stance_transverse: distance from foot to hip in transverse direction. Typically the hip length.

        Returns: (vertical, forward, transverse) position of foot with respect to leg base frame
        """
        vertical_to_leg_base = self.vertical_wrt_leg_base(foot_height)
        transverse_to_leg_base = self.transverse_wrt_leg_base(foot_transverse)
        sagittal_to_leg_base = self.sagital_wrt_leg_base(foot_forward)
        return [vertical_to_leg_base, transverse_to_leg_base, sagittal_to_leg_base]

    def target_foot_thetas(self, foot_name, vertical, transverse, sagittal):
        """
        Calculate target foot joint angles (thetas) based on foot position.

        Inputs:
        foot_name: name of the foot/leg (e.g., 'LF', 'RF', 'LB', 'RB')
        vertical: vertical position of foot with respect to leg base frame
        transverse: transverse position of foot with respect to leg base frame
        sagittal: sagittal position of foot with respect to leg base frame

        Returns: (theta_x, theta_y, theta_z) joint angles for the foot
        """

        # 
        X, Y, Z = self.foot_position_to_base(foot_name, vertical, transverse, sagittal)

        if foot_name in ['LF', 'LB']:
            dh_table = self.robot.dh_left
        else:
            dh_table = self.robot.dh_right

        [theta1, theta2, theta3] = kin.inverse_kinematics(
            dh_table,
            [X, Y, Z],
            initial_guess=self.robot.neutral_stance_thetas[foot_name],
            var_indices=[0, 2, 3],
            fixed_values={1: np.pi},
        )

        return theta1, theta2, theta3
    
    def foot_position_to_base(self, foot_name, vertical, transverse, sagittal):
        """
        Determine foot position with respect to leg base frame (i.e., with direction)
        
        Return
        -------
        leg_base_x, leg_base_y, leg_base_z: foot position with respect to leg base frame
        """
        if foot_name in ['LF', 'LB']:
            return -vertical, -transverse, sagittal
        else:
            return -vertical, transverse, sagittal


    def target_feet_thetas(self, feet_displacements):

        '''
        
        Returns
        -------
         dict[str, list[float]]
            target foot joint angles for each leg. List is [theta1, theta2, theta3]
        '''

        feet_thetas = {}

        for foot in self.robot.leg_names:
            X_disp, Y_disp, Z_disp = (
                feet_displacements.loc[foot, "Vertical"],
                feet_displacements.loc[foot, "Transverse"],
                feet_displacements.loc[foot, "Forward"],
            )
            X, Y, Z = self.target_foot_position_wrt_base(
                foot_height=X_disp,
                foot_transverse=Y_disp,
                foot_forward=Z_disp,
            )

            # print(f"Displacements: {feet_displacements.loc[foot]}")
            # print(f"Foot: {foot}, X: {X}, Y: {Y}, Z: {Z}")  # Debug print
            
            
            feet_thetas[foot] = self.target_foot_thetas(foot, X, Y, Z)

        return feet_thetas

    def mode_receiver(self, msg):
        mode = msg.data
        if mode == 'walk' and not self.in_walk:
            self.start_walking()
        elif mode == 'idle' and self.in_walk:
            self.start_idle()
    
    def start_walking(self):
        self.in_walk = True
        self.walk_start_time = (Clock(clock_type=ClockType.STEADY_TIME).now())
        # Start walking logic here
        
    def start_idle(self):
        self.in_walk = False
        # Stop walking logic here

    def control_tick(self):

        '''
        
        Publish joint angles based on current mode (walking or idle).
        
        joint_angles: [LF_yaw, LF_pitch, LF_knee, RF_yaw, RF_pitch, RF_yaw,...] list of joint angles to publish
        
        '''        
        
        if not self.in_walk:
            feet_thetas = self.robot.neutral_stance_thetas
        else:
            feet_displacements = self.walk.get_feet_displacements(self.walk_start_time)
            feet_thetas = self.target_feet_thetas(feet_displacements)
        
        joint_angles = [angle for leg in self.robot.leg_names for angle in feet_thetas[leg]] # feet thetas but in list form to match ordering as per joint names in robotconfig
        print("Joint Angles:")
        print(joint_angles[:3])
        print(joint_angles[3:6])
        print(joint_angles[6:9])
        print(joint_angles[9:])
        print('\n')

        # print(self.robot.joint_names)
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.robot.joint_names
        msg.position = joint_angles
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    robot_config = robot_config_.RobotConfig()
    walk = walk_.Walk(robot_config)

    node = GaitController(robot_config, walk)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 