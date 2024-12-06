import math
from typing import List, Tuple
from pid import PIDController
from sr.robot3 import Robot

class PositioningRobot(Robot):
    def __init__(self, zone=0):
        super().__init__()
        self.zone = zone
        self.linear_pid = PIDController(sp=0, p=0.5, i=0.01, d=0.1)   # Example PID gains (To be tuned)
        self.angular_pid = PIDController(sp=0, p=0.5, i=0.01, d=0.1)  # Example PID gains (To be tuned)
        # Known obstacles: list of (x_min, y_min, x_max, y_max)
        # This is a placeholder.
        self.obstacles = [
            (2.0, 1.5, 2.5, 2.0),  # Example obstacle
            # Add highrises here
        ]
    
    def is_path_clear(self, start_x: float, start_y: float, end_x: float, end_y: float) -> bool:
        """
        Checks if the straight line path from (start_x, start_y) to (end_x, end_y)
        intersects any known obstacle.
        
        Obstacles are axis-aligned rectangles defined by (x_min, y_min, x_max, y_max).
        This is a simple line-rectangle intersection check.
        
        Parameters:
            start_x, start_y: start of line segment
            end_x, end_y: end of line segment
        
        Returns:
            bool: True if path is clear, False if blocked.
        """
        for (ox_min, oy_min, ox_max, oy_max) in self.obstacles:
            if self._line_intersects_rect(start_x, start_y, end_x, end_y, ox_min, oy_min, ox_max, oy_max):
                return False
        return True
    
    @staticmethod
    def _line_intersects_rect(x1, y1, x2, y2, rx1, ry1, rx2, ry2):
        """
        Checks if the line segment from (x1, y1) to (x2, y2) intersects
        the axis-aligned rectangle defined by (rx1, ry1, rx2, ry2).
        
        Returns True if they intersect, False otherwise.
        """
        # Check if either endpoint is inside the rectangle
        if (rx1 <= x1 <= rx2 and ry1 <= y1 <= ry2) or (rx1 <= x2 <= rx2 and ry1 <= y2 <= ry2):
            return True
        
        # Check intersection with each side of the rectangle
        return (PositioningRobot._line_intersect(x1, y1, x2, y2, rx1, ry1, rx1, ry2) or
                PositioningRobot._line_intersect(x1, y1, x2, y2, rx2, ry1, rx2, ry2) or
                PositioningRobot._line_intersect(x1, y1, x2, y2, rx1, ry1, rx2, ry1) or
                PositioningRobot._line_intersect(x1, y1, x2, y2, rx1, ry2, rx2, ry2))
    
    @staticmethod
    def _line_intersect(x1, y1, x2, y2, x3, y3, x4, y4):
        """
        Checks if line segment (x1,y1)-(x2,y2) intersects line segment (x3,y3)-(x4,y4).
        Returns True if intersecting, False otherwise.
        """
        denom = (y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1)
        if denom == 0:
            return False  # Parallel or coincident lines
        ua = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3))/denom
        ub = ((x2 - x1)*(y1 - y3) - (y2 - y1)*(x1 - x3))/denom
        return 0 <= ua <= 1 and 0 <= ub <= 1

    def move_to_pose(self, target_x: float, target_y: float, target_theta: float,
                     linear_tolerance=0.05, angular_tolerance=0.05, max_iterations=200):
        """
        Moves the robot to the desired pose (target_x, target_y, target_theta) using PID control.
        
        Parameters:
            target_x (float): Target x-coordinate (m)
            target_y (float): Target y-coordinate (m)
            target_theta (float): Target orientation (radians)
            linear_tolerance (float): Tolerance for position in meters.
            angular_tolerance (float): Tolerance for orientation in radians.
            max_iterations (int): Maximum control loop iterations to prevent infinite loops.
        """
        
        # Reset PIDs with new setpoints
        # Linear PID controls distance to target
        # Angular PID controls heading to target
        self.linear_pid.reset(sp=0)
        self.angular_pid.reset(sp=0)
        
        for i in range(max_iterations):
            # Get current pose
            pos = self.get_pos()
            if not pos:
                # Error handling because you never know
                continue
            rx, ry, rtheta = pos

            # Compute error
            dx = target_x - rx
            dy = target_y - ry
            dist_error = math.sqrt(dx*dx + dy*dy)
            
            # Desired angle to target
            desired_heading = math.atan2(dy, dx)
            heading_error = self._angle_diff(desired_heading, rtheta)

            # Check if close enough
            if dist_error < linear_tolerance and abs(self._angle_diff(target_theta, rtheta)) < angular_tolerance:
                # We reached the goal
                self.power((0.0, 0.0), 0)  # Stop
                return True
            
            # If direct path is not clear, attempt a simple avoidance:
            if not self.is_path_clear(rx, ry, target_x, target_y):
                # Simple avoidance: rotate in place slightly and proceed
                # This is a naive approach, real solution might require path planning.
                self.power((0.3, -0.3), 0.5, off=True)  # turn on the spot
                continue
            
            # Set PID setpoints:
            self.linear_pid.sp = 0  # We want to reduce distance error to 0
            self.angular_pid.sp = 0 # We want to reduce heading error to 0

            # Get corrections
            linear_correction = self.linear_pid.getCorrection(dist_error)
            angular_correction = self.angular_pid.getCorrection(heading_error)

            # Combine into wheel commands
            # For differential drive:
            # left_wheel = linear_speed - angular_speed
            # right_wheel = linear_speed + angular_speed
            linear_speed = self._constrain(linear_correction, -0.5, 0.5)  # limit speed
            angular_speed = self._constrain(angular_correction, -0.5, 0.5)
            
            left_power = linear_speed - angular_speed
            right_power = linear_speed + angular_speed

            # Duration small to allow iterative control loop
            self.power((left_power, right_power), 0.1, off=True)

        # If we exit the loop, we didn't reach the goal in time
        return False

    @staticmethod
    def _angle_diff(a, b):
        """
        Computes the minimal difference between two angles a and b.
        Result is in (-pi, pi).
        """
        diff = (a - b) % (2*math.pi)
        if diff > math.pi:
            diff -= 2*math.pi
        return diff

    @staticmethod
    def _constrain(value, min_val, max_val):
        return max(min_val, min(max_val, value))
