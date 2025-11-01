import time
import numpy as np
import pandas as pd
import yaml
import os
import kinematics as kin

import rclpy
from rclpy.node import Node


#### TO DO
# - ADD ROS2
# - SPIN UP NODES FOR KINEMATICS AND MOPTION PLANNING
# - HOW TO INTEGRATE TO HARDWARE?


class RobotConfig:
    hip_length: float
    thigh_length: float
    shin_length: float

    body_length: float
    body_width: float
    body_height: float

    max_step_height: float
    max_step_length: float
    deg_per_sec: float

    neutral_stance_thetas: list[float]
    neutral_stance_height: float

    def __init__(self):
        self.params = None

        params = self.load_parameters()
        self.assign_parameters(params)

    def load_parameters(self):
        # Locate and load the config file
        config_path = os.path.join("../", "config", "robot_dimensions.yaml")

        with open(config_path, "r") as f:
            return yaml.safe_load(f)

    def assign_parameters(self, params):
        # leg params
        self.hip_length = params["leg"]["link1_length"]
        self.thigh_length = params["leg"]["link2_length"]
        self.shin_length = params["leg"]["link3_length"]

        # body params
        self.body_length = params["body"]["length"]
        self.body_width = params["body"]["width"]
        self.body_height = params["body"]["height"]

        # neutral stance params
        neutral_stance_height_pct = params["neutral_stance"][
            "neutral_stance_height_pct"
        ]
        self.neutral_stance_height = self.calc_neutral_stance_height(
            neutral_stance_height_pct
        )
        self.neutral_stance_thetas = self.calc_neutral_stance_thetas(
            self.neutral_stance_height,
            [self.hip_length, self.thigh_length, self.shin_length],
        )

        # walking params
        max_step_height_pct = params["walking"]["max_step_height_pct"]
        step_length_deg = params["walking"]["step_length_deg"]
        self.deg_per_sec = params["walking"]["deg_per_sec"]
        self.max_step_height = self.calc_max_step_height(max_step_height_pct)
        self.max_step_length = self.calc_max_step_length(step_length_deg)

    def calc_neutral_stance_height(self, neutral_stance_height_pct):
        # Calculate neutral height from hip axis to ground
        # ASSUMING theta1 = 0 (no hip rotation)
        return (self.thigh_length + self.shin_length) * neutral_stance_height_pct

    def calc_max_step_height(self, max_step_height_pct):
        # Example calculation based on leg lengths
        return (self.neutral_stance_height) * max_step_height_pct

    def calc_max_step_length(self, step_length_deg):
        # Example calculation based on leg lengths
        return np.sin(np.radians(step_length_deg)) * (
            self.thigh_length + self.shin_length
        )

    def calc_neutral_stance_thetas(self, neutral_stance_height, link_lengths):
        # Use inverse kinematics to calculate neutral stance joint angles
        target_pos = [
            -neutral_stance_height,  # x axis is vertical (positive up)
            link_lengths[0],  # y axis is transverse (positive outwards)
            0,  # z axis is sagittal (positive forward)...0 is directly under hip
        ]
        return kin.inverse_kinematics(
            link_lengths,
            target_pos,
            initial_guess=[0, -45, 90],
        )


class MotionPlanning:
    def __init__(self, robotconfig):
        self.robot = robotconfig

    def vertical_wrt_base(self, neutral_stance_height, foot_height):
        """
        Calculate vertical position of foot with respect to base frame.

        Base frame is located at hip joint.

        foot_height: vertical position of foot with respect to ground
        neutral_stance_height: height of body frame with respect to ground in neutral stance

        Returns: vertical position of foot with respect to base frame
        """
        return neutral_stance_height - foot_height

    def sagital_wrt_base(self, neutral_stance_forward, foot_forward):
        """
        Calculate sagittal position of foot with respect to base frame. (forward/backward)

        Base frame is located at hip joint.

        foot_forward: sagittal position of foot with respect to body center
        neutral_stance_forward: distance from foot to hip in sagittal direction. Typically 0.

        Returns: sagittal position of foot with respect to base frame
        """
        return neutral_stance_forward + foot_forward

    def transverse_wrt_base(self, neutral_stance_transverse, foot_transverse):
        """
        Calculate transverse position of foot with respect to base frame. (left/right)

        Base frame is located at hip joint.

        foot_transverse: transverse position of foot with respect to body center
        neutral_stance_transverse: distance from foot to hip in transverse direction. Typically the hip length.

        Returns: transverse position of foot with respect to base frame
        """
        return neutral_stance_transverse + foot_transverse

    def target_foot_position_wrt_base(
        self,
        neutral_stance_height,
        neutral_stance_forward,
        neutral_stance_transverse,
        foot_height,
        foot_forward,
        foot_transverse,
    ):
        """
        Calculate foot position with respect to base frame.

        Base frame is located at hip joint.

        foot_height: vertical position of foot with respect to ground
        foot_forward: sagittal position of foot with respect to body center
        foot_transverse: transverse position of foot with respect to body center
        neutral_stance_height: height of body frame with respect to ground in neutral stance
        neutral_stance_forward: distance from foot to hip in sagittal direction. Typically 0.
        neutral_stance_transverse: distance from foot to hip in transverse direction. Typically the hip length.

        Returns: (vertical, forward, transverse) position of foot with respect to base frame
        """
        X = self.vertical_wrt_base(neutral_stance_height, foot_height)
        Z = self.sagital_wrt_base(neutral_stance_forward, foot_forward)
        Y = self.transverse_wrt_base(neutral_stance_transverse, foot_transverse)
        return X, Y, Z

    def target_foot_thetas(self, X, Y, Z):
        """
        Calculate target foot joint angles (thetas) based on foot position.

        Returns: (theta_x, theta_y, theta_z) joint angles for the foot
        """

        [theta1, theta2, theta3] = kin.inverse_kinematics(
            [self.robot.hip_length, self.robot.thigh_length, self.robot.shin_length],
            [X, Y, Z],
            initial_guess=self.robot.neutral_stance_thetas,
        )

        return theta1, theta2, theta3


class Walk:
    def __init__(self):
        self.deg_per_sec = 10  # joint speed in degrees per second
        self.max_foot_height = 2  # max foot lift height
        self.max_foot_forward = 2

        # foot deg offsets
        self.phase_offsets = {
            "LF": 0,
            "RF": 180,
            "RB": 0,
            "LB": 180,
        }

    def walk(self):
        start_time = time.time()
        feet_positions = pd.DataFrame(
            index=self.phase_offsets.keys(),
            columns=["Foot", "Vertical", "Forward", "Transverse"],
        )

        while True:
            self.get_feet_positions(start_time, feet_positions)
            time.sleep(0.1)

    def get_loop_deg(self, start_time):
        return (time.time() - start_time) * self.deg_per_sec

    def get_feet_positions(self, start_time, feet_positions):
        for foot, phase_offset in self.phase_offsets.items():
            remainder_deg = (self.get_loop_deg(start_time) + phase_offset) % 360

            feet_positions.loc[foot, "Vertical"] = self.get_vertical(remainder_deg)
            feet_positions.loc[foot, "Forward"] = self.get_forward(remainder_deg)
            feet_positions.loc[foot, "Transverse"] = self.get_transverse()
        return feet_positions

    def get_vertical(self, remainder_deg):
        """
        Calculate vertical foot position based on remainder degrees.

        remainder_deg: degrees within the current step cycle (0-360)
        Returns: vertical position (float)

        """

        if remainder_deg < 180:
            return self.max_foot_height * np.sin(np.radians(remainder_deg))
        else:
            return float(0)

    def get_forward(self, remainder_deg):
        # Example implementation

        if remainder_deg > 180:
            return self.max_foot_forward * remainder_deg / 180
        else:
            return self.max_foot_forward * (1 - (remainder_deg % 180) / 180)

    def get_transverse(self):
        # Placeholder for transverse movement calculation
        # No transverse movement in current model
        return float(0)

class ModeManager:
    def __init__(self):
        self.current_mode = "IDLE"

    def set_mode(self, mode):
        self.current_mode = mode

    def get_mode(self):
        return self.current_mode