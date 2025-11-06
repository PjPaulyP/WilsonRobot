import numpy as np
import pandas as pd
import yaml
import os
from robot_pkg import kinematics as kin
from ament_index_python.packages import get_package_share_directory

class RobotConfig:
        
    hip_length: float
    thigh_length: float
    shin_length: float

    body_length: float
    body_width: float
    body_height: float

    # walking params
    max_step_height: float
    max_step_length: float
    deg_per_sec: float
    
    leg_names: list[str]
    phase_offsets: dict[str, float]
    joint_names: list[str]
    
    neutral_stance_thetas: list[float]
    neutral_stance_height: float
    neutral_stance_forward: float
    neutral_stance_transverse: float
    
    control_hz: float
    
    dh_left: list[dict]
    dh_right: list[dict]

    def __init__(self):
        self.params = None

        params = self.load_parameters()
        self.assign_parameters(params)

    def load_parameters(self):
        # Locate and load the config file
        pkg_share = get_package_share_directory('robot_pkg') 
        config_path = os.path.join(pkg_share, "config", "robot_dimensions.yaml")

        with open(config_path, "r") as f:
            return yaml.safe_load(f)

    def assign_parameters(self, params):
               
        # leg and joint names 
        self.joint_names = []
        self.leg_names = params["leg"]["leg_acronyms"] # LF = left front, RF = right front, LB = left back, RB = right back
        angle_names = ['theta1', 'theta2', 'theta3']
        for leg in self.leg_names:
            for angle in angle_names:
                self.joint_names.append(f"{leg}_{angle}")
                
        self.control_hz = params["control"]["control_hz"]
        
        # leg params
        self.hip_length = params["leg"]["link1_length"]
        self.thigh_length = params["leg"]["link2_length"]
        self.shin_length = params["leg"]["link3_length"]

        # body params
        self.body_length = params["body"]["length"]
        self.body_width = params["body"]["width"]
        self.body_height = params["body"]["height"]

        L1 = self.hip_length
        L2 = self.thigh_length
        L3 = self.shin_length

        self.dh_right = [
            {'a': 0.0, 'alpha': -np.pi/2, 'd': 0.0, 'theta': None},   # joint 0 -> theta variable t1
            {'a': 0.0, 'alpha': 0.0,       'd': L1, 'theta': np.pi},  # joint 1 fixed at 180deg
            {'a': L2,  'alpha': 0.0,       'd': 0.0, 'theta': None},   # joint 2 variable t2
            {'a': L3,  'alpha': 0.0,       'd': 0.0, 'theta': None},   # joint 3 variable t3 (end effector)
        ]

        self.dh_left = [
            {'a': 0.0, 'alpha': -np.pi/2, 'd': 0.0, 'theta': None},   # joint 0 -> theta variable t1
            {'a': 0.0, 'alpha': 0.0,       'd': L1, 'theta': np.pi},  # joint 1 fixed at 180deg
            {'a': L2,  'alpha': 0.0,       'd': 0.0, 'theta': None},   # joint 2 variable t2
            {'a': L3,  'alpha': 0.0,       'd': 0.0, 'theta': None},   # joint 3 variable t3 (end effector)
        ]

        # neutral stance params
        neutral_stance_height_pct = params["neutral_stance"][
            "neutral_stance_height_pct"
        ]       
        self.neutral_stance_height, self.neutral_stance_transverse, self.neutral_stance_forward = self.calc_neutral_stance_displacements(
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
        self.phase_offsets = {}
        for leg in self.leg_names:
            self.phase_offsets[leg] = params["walking"][f"{leg}_phase_offset"]


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
        
        '''
        
        Returns
        -------
        dict[str, list[float]]
            neutral stance joint angles for each leg
        '''
        
        # Use inverse kinematics to calculate neutral stance joint angles
        target_pos = [
            -neutral_stance_height,  # x axis is vertical (positive up)
            link_lengths[0],  # y axis is transverse (positive outwards)
            0,  # z axis is sagittal (positive forward)...0 is directly under hip
        ]
        
        
        neutral_feet_thetas = {}

        for foot in self.leg_names:

            if foot in ['LF', 'LH']:
                dh_table = self.dh_left
            else:
                dh_table = self.dh_right

            neutral_feet_thetas[foot] = kin.inverse_kinematics(
                dh_table,
                target_pos,
                initial_guess=np.radians([0, -45, 90]),
                var_indices=[0, 2, 3],
                fixed_values={1: np.pi},
            )

        return neutral_feet_thetas

    def calc_neutral_stance_displacements(self, neutral_stance_height_pct):
        # Use forward kinematics to calculate neutral stance foot displacements

        neutral_stance_height = self.calc_neutral_stance_height(neutral_stance_height_pct)
        neutral_stance_transverse = self.hip_length  # y axis
        neutral_stance_forward = 0  # z axis    

        return neutral_stance_height, neutral_stance_transverse, neutral_stance_forward
    


