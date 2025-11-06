import numpy as np
import pandas as pd
from rclpy.clock import Clock, ClockType



class Walk():
    def __init__(self, robot_config):
        self.deg_per_sec = robot_config.deg_per_sec  # joint speed in degrees per second
        self.max_step_height = robot_config.max_step_height  # max foot lift height
        self.max_step_length = robot_config.max_step_length  # max foot forward position
        self.phase_offsets = robot_config.phase_offsets

    def get_loop_deg(self, elapsed_time):
        # elapsed_time may be an rclpy Duration object (result of Clock.now() - start_time)
        # elapsed time is a Clock object in nanoseconds
        
        seconds = float(elapsed_time.nanoseconds) * 1e-9

        return seconds * self.deg_per_sec

    def get_feet_displacements(self, start_time):
        
        elapsed_time = (Clock(clock_type=ClockType.STEADY_TIME).now()) - start_time
        
        feet_displacements = pd.DataFrame(
            index=self.phase_offsets.keys(),
            columns=["Foot", "Vertical", "Forward", "Transverse"],
        )
        
        for foot, phase_offset in self.phase_offsets.items():
            remainder_deg = (self.get_loop_deg(elapsed_time) + phase_offset) % 360

            feet_displacements.loc[foot, "Vertical"] = self.get_vertical(remainder_deg)
            feet_displacements.loc[foot, "Forward"] = self.get_forward(remainder_deg)
            feet_displacements.loc[foot, "Transverse"] = self.get_transverse()
        return feet_displacements

    def get_vertical(self, remainder_deg):
        """
        Calculate vertical foot position based on remainder degrees.

        remainder_deg: degrees within the current step cycle (0-360)
        Returns: vertical position (float)

        """

        if remainder_deg < 180:
            return self.max_step_height * np.sin(np.radians(remainder_deg))
        else:
            return float(0)

    def get_forward(self, remainder_deg):
        # Example implementation

        if remainder_deg < 180:
            return self.max_step_length * remainder_deg / 180
        else:
            return self.max_step_length * (1 - (remainder_deg % 180) / 180)

    def get_transverse(self):
        # Placeholder for transverse movement calculation
        # No transverse movement in current model
        return float(0)


