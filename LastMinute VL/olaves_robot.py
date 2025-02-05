#Jawad Uddin, 04/02/2025
#olaves_robot.py

import sr.robot3 as sr
from typing import Optional, List, Tuple
from math import pi, sin, cos, atan2, sqrt, tan, floor, degrees, atan
import numpy as np
from time import time
from random import randint

class OlavesRobot(sr.Robot):
    def __init__(self):
        super().__init__()
        self.myPallets = set(range(100 + (20*self.zone), 120 + (20*self.zone)))
        self.enemyPallets = set(range(100,180)).difference(self.myPallets)
        self.init_time: float = time()
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
        elapsed_time = time() - self.init_time
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


    def calculate_heat_level(self, pitch, distance) -> float:
        pass