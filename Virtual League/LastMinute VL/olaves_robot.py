#Jawad Uddin, 04/02/2025
#olaves_robot.py

import sr.robot3 as sr
from typing import Optional, List, Tuple
from math import pi, sin, cos, atan2, sqrt, tan, floor, degrees, atan
import numpy as np
from random import randint
from pid import PIDController

class OlavesRobot(sr.Robot):
    def __init__(self):
        super().__init__()
        self.myPallets = set(range(100 + (20*self.zone), 120 + (20*self.zone)))
        self.enemyPallets = set(range(100,180)).difference(self.myPallets)
        self.hiRises = set(range(195,200))
        self.init_time: float = self.time()
        #motor board: self.motor_board
        #servo board: self.servo_board
        #power board: self.power_board
        #arduino board: self.arduino
    

    def get_comp_time(self) -> float:
        """
        Returns the time elapsed since the robot was initialised.

        Computes the difference between the current time and the initialisation time.

        Returns:
            float: Elapsed time in seconds.
        """
        elapsed_time = self.time() - self.init_time
        print(f"Computed elapsed time: {elapsed_time} seconds.")
        return elapsed_time


    def sleep(self, duration: float) -> None:
        """
        Pauses execution for a specified duration.

        Acts as time.sleep, but tailored for the simulation environment where robot.sleep() may not function as expected.

        Parameters:
            duration (float): The amount of time to sleep in seconds.
        """
        print(f"Sleeping for {duration} seconds.")
        super().sleep(duration)


    def turn(self, angle: float) -> None:
        """
        Performs a dead reckoning turn.

        Calculates the duration required to turn the specified angle based on a predefined multiplier and adjusts motor powers accordingly.

        Parameters:
            angle (float): The angle to turn in radians. Positive values indicate a left turn, while negative values indicate a right turn.
        """
        ANGLE_TO_TIME = 14.2  # Multiplier to make the turn more accurate
        direction = "left" if angle > 0 else "right"
        print(f"Turning {direction} by {abs(angle)} radians.")

        if angle > 0:
            # Turn left
            self.power((0.4, -0.4), angle / pi * ANGLE_TO_TIME)
        else:
            # Turn right
            self.power((-0.4, 0.4), abs(angle) / pi * ANGLE_TO_TIME)
        print(f"Completed turn {direction}.")


    def move(self, distance: float) -> None:
        """
        Performs a dead reckoning move.

        Calculates the duration required to move the specified distance based on a predefined multiplier and adjusts motor powers accordingly.

        Parameters:
            distance (float): The distance to move in meters. Positive values move the robot forward, while negative values move it backward.
        """
        DIST_TO_TIME = 5
        direction = "forward" if distance > 0 else "backward"
        print(f"Moving {direction} by {abs(distance)} meters.")

        if distance > 0:
            self.power((0.4, 0.4), distance * DIST_TO_TIME)
        else:
            self.power((-0.4, -0.4), abs(distance) * DIST_TO_TIME)
        print(f"Completed move {direction}.")
    

    def power(
        self,
        wheels: Tuple[float, float],
        duration: float,
        off: bool = True,
    ) -> None:
        """
        Controls the motors by setting their power levels.

        Adjusts the power of the left and right motors for a specified duration. Optionally turns off the motors after the duration.

        Parameters:
            wheels (Tuple[float, float]):
                - First element controls the left wheel's power.
                - Second element controls the right wheel's power.
                Positive values move the wheels forward, while negative values move them backward.
            duration (float): The duration in seconds for which the motors should run.
            off (bool, optional): Whether to turn off the motors after the duration. Defaults to True.

        Raises:
            ValueError: If wheel power values are outside the allowable range after correction.
        """
        self.motor_board.motors[1].power = self._clamp(wheels[1])
        self.motor_board.motors[0].power = self._clamp(wheels[0])
        self.sleep(duration)
        if off:
            self.motor_board.motors[0].power = 0.0
            self.motor_board.motors[1].power = 0.0


    @staticmethod
    def _clamp(value: float, min_value: float = -1.0, max_value: float = 1.0) -> float:
        """
        Clamps a value to the specified range.

        Ensures that the provided value does not exceed the defined minimum and maximum boundaries.

        Parameters:
            value (float): The value to clamp.
            min_value (float, optional): The minimum allowable value. Defaults to -1.0.
            max_value (float, optional): The maximum allowable value. Defaults to 1.0.

        Returns:
            float: The clamped value.
        """
        clamped_value = max(min_value, min(max_value, value))
        return clamped_value
    

    def get_closest_pallet(self, enemy = False) -> object:
        try:
            markers = self.camera.see()
            shortestDistance = 10e9
            closestPallet = None
            thePallets = self.myPallets
            if enemy == True:
                thePallets = self.enemyPallets
            for marker in markers:
                if marker.id in thePallets and marker.distance < shortestDistance:
                    closestPallet = marker
                    shortestDistance = marker.distance
            return closestPallet
        except:
            return None
    
    def get_closest_hirise(self) -> object:
        try:
            markers = self.camera.see()
            shortestDistance = 10e9
            closestHirise = None
            for marker in markers:
                if marker.id in self.hiRises and marker.distance < shortestDistance:
                    closestHirise = marker
                    shortestDistance = marker.distance
            return closestHirise
        except:
            return None

    def reverse_and_reset(self) -> None:
        self.move(-0.4)
        self.sleep(0.5)
        self.turn(randint(0,1) - pi/2)
        self.sleep(0.25)
    

    def pick_up_box(self) -> None:
        self.servo_board.servos[0].position = -1
        self.sleep(0.25)
        self.power_board.outputs[OUT_H0].is_enabled = True
        self.sleep(0.25)
        self.servo_board.servos[0].position = 1
        self.sleep(0.25)


    def drop_box(self) -> None:
        self.servo_board.servos[0].position = 0
        self.sleep(0.25)
        self.power_board.outputs[OUT_H0].is_enabled = False
        self.sleep(0.25)

    def move_to_marker(self, marker_id: int) -> None:
        """
        Moves the robot to a specified marker using PID controllers.

        Parameters:
            marker_id (int): The ID of the target marker.
        """
        # logger.info(f"Moving to Marker: {marker_id}")

        DIST_TOLERANCE = 0.1  # meters
        ANGLE_TOLERANCE = 0.1  # radians
        DIST_TO_TIME = 0.05  # Base duration for movement

        markers = [m for m in self.camera.see() if m.id == marker_id]
        if not markers:
            # logger.error(f"Marker {marker_id} not seen initially.")
            # TODO: Add some handling logic for when a marker can't be seen
            return

        marker = sorted(markers, key=lambda x: x.position.distance)[0]
        init_dist = marker.position.distance / 1000.0  # Convert mm to meters

        dist_error = init_dist
        angle_error = marker.position.horizontal_angle

        # Initialise PID controllers
        # Distance PID: Only P and I to prevent aggressive changes
        dist_pid = PIDController(
            sp=0.0, p=0.15, i=0.05, d=0.05, integrator_min=-5.0, integrator_max=5.0
        )

        # Angle PID: P and D for responsive turning with damping
        angle_pid = PIDController(
            sp=0.0, p=0.4, i=0.2, d=0.3, integrator_min=-5.0, integrator_max=5.0
        )

        iterations = 0
        counter = 0
        while (
            (dist_error > DIST_TOLERANCE or abs(angle_error) > ANGLE_TOLERANCE)
            and iterations != 1000
            and counter < 5
        ):
            # Calculate PID corrections
            dist_correction = -dist_pid.getCorrection(dist_error)
            angle_correction = angle_pid.getCorrection(angle_error)

            # Convert corrections into left/right motor power
            left_power = dist_correction - angle_correction
            right_power = dist_correction + angle_correction

            # Apply scaling with minimum power
            left_power, right_power = self._scale(
                left_power, right_power, min_power=0.05
            )

            # Determine dynamic time_step based on distance error
            # This can help in smoother control as the robot approaches the target
            time_step = min(
                max((dist_error / init_dist) * DIST_TO_TIME, 0.1 * DIST_TO_TIME),
                DIST_TO_TIME,
            )

            # Apply motor power
            self.power((left_power, right_power), time_step, off=False)

            # Update marker information
            markers = [m for m in self.camera.see() if m.id == marker_id]
            if markers:
                marker = markers[0]
                counter = 0
            else:
                self.power((0.0, 0.0), 0.0)
                counter += 1
                if self.arduino.ultrasound_measure(2, 3) < DIST_TOLERANCE:
                    self._move_to_marker_finalisation()
                else:
                    pass

            # Update errors
            dist_error = marker.position.distance / 1000.0
            angle_error = marker.position.horizontal_angle

            iterations += 1

        # Stop the robot once target is reached
        self.power((0.0, 0.0), 0.0)

    def _move_to_marker_finalisation(self) -> None:
        DIST_TOLERANCE = 0.1  # meters
        DIST_TO_TIME = 0.01  # Base duration for movement

        init_dist = (
            self.arduino.ultrasound_measure(2, 3) / 1000.0
        )  # Convert mm to meters

        dist_error = init_dist

        # Initialise PID controllers
        # Distance PID: Only P and I to prevent aggressive changes
        dist_pid = PIDController(
            sp=0.0, p=0.2, i=0.05, d=0.05, integrator_min=-5.0, integrator_max=5.0
        )

        iterations = 0
        while dist_error > DIST_TOLERANCE and iterations != 100:
            # Calculate PID corrections
            dist_correction = -dist_pid.getCorrection(dist_error)

            # Convert corrections into left/right motor power
            left_power = right_power = dist_correction

            # Apply scaling with minimum power
            left_power, right_power = self._scale(
                left_power, right_power, min_power=0.05, center=0.1
            )

            # Determine dynamic time_step based on distance error
            # This can help in smoother control as the robot approaches the target
            time_step = min(
                max((dist_error / init_dist) * DIST_TO_TIME, 0.1 * DIST_TO_TIME),
                DIST_TO_TIME,
            )

            # Apply motor power
            self.power((left_power, right_power), time_step, off=False)


    def checkVacuumEnabled(self):
        try:
            return self.power_board.outputs[OUT_H0].is_enabled
        except:
            return False