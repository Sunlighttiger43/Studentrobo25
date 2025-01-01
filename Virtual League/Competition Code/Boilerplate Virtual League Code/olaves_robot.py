import sr.robot3 as sr
from math import pi, sin, cos, atan2, asin, tan
from typing import List, Tuple, Optional
from pid import PIDController
import numpy as np
from scipy.optimize import least_squares
from time import time
import pickle
import logging
import heapq

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class OlavesRobot(sr.Robot):
    def __init__(self):
        logger.info("Initialising OlavesRobot")
        self.init_time: float = time()

    def get_comp_time(self) -> float:
        """
        Returns the time elapsed since the robot was initialised.

        Computes the difference between the current time and the initialisation time.

        Returns:
            float: Elapsed time in seconds.
        """
        elapsed_time = time() - self.init_time
        logger.debug(f"Computed elapsed time: {elapsed_time} seconds.")
        return elapsed_time

    def sleep(self, duration: float) -> None:
        """
        Pauses execution for a specified duration.

        Acts as time.sleep, but tailored for the simulation environment where robot.sleep() may not function as expected.

        Parameters:
            duration (float): The amount of time to sleep in seconds.
        """
        logger.debug(f"Sleeping for {duration} seconds.")
        super().sleep(duration)

    def _turn(self, angle: float) -> None:
        """
        Performs a dead reckoning turn.

        This method should not be used outside the OlavesRobot subclass. It calculates the duration required to turn the specified angle
        based on a predefined multiplier and adjusts motor powers accordingly.

        Parameters:
            angle (float): The angle to turn in radians. Positive values indicate a left turn, while negative values indicate a right turn.
        """
        ANGLE_TO_TIME = 14.2  # Multiplier to make the turn more accurate
        direction = "left" if angle > 0 else "right"
        logger.info(f"Turning {direction} by {abs(angle)} radians.")

        if angle > 0:
            # Turn left
            self.power((0.4, -0.4), angle / pi * ANGLE_TO_TIME)
        else:
            # Turn right
            self.power((-0.4, 0.4), abs(angle) / pi * ANGLE_TO_TIME)
        logger.info(f"Completed turn {direction}.")

    def _move(self, distance: float) -> None:
        """
        Performs a dead reckoning move.

        This method should not be used outside the OlavesRobot subclass. It calculates the duration required to move the specified distance
        based on a predefined multiplier and adjusts motor powers accordingly.

        Parameters:
            distance (float): The distance to move in meters. Positive values move the robot forward, while negative values move it backward.
        """
        DIST_TO_TIME = 5
        direction = "forward" if distance > 0 else "backward"
        logger.info(f"Moving {direction} by {abs(distance)} meters.")

        if distance > 0:
            self.power((0.4, 0.5), distance * DIST_TO_TIME)
        else:
            self.power((-0.4, -0.5), abs(distance) * DIST_TO_TIME)
        logger.info(f"Completed move {direction}.")

    def get_marker_orientation(self, marker_id: int) -> float:
        """
        Returns the absolute orientation of a specified marker.

        This method is not needed for the virtual league and is intentionally left unimplemented.

        Assumptions:
            - To be based on new positioning system proposed by Kushal and Francis

        Parameters:
            marker_id (int): The ID of the marker.

        Raises:
            NotImplementedError: Always raised as this method is not required for the virtual league.
        """
        logger.warning(
            "get_marker_orientation is not implemented for the virtual league."
        )
        raise NotImplementedError(
            "get_marker_orientation is not needed for the virtual league."
        )

    def _get_marker_by_id(self, marker_id: int) -> sr.Marker:
        """
        Retrieves a marker by its ID.

        Searches the robot's camera feed for a marker with the specified ID.

        Parameters:
            marker_id (int): The ID of the marker to retrieve.

        Returns:
            sr.Marker: The marker object corresponding to the given ID.

        Raises:
            ValueError: If no marker with the specified ID is found.
        """
        marker = next((m for m in self.camera.see() if m.id == marker_id), None)
        if marker is None:
            logger.error(f"Marker with ID {marker_id} not found.")
            raise ValueError(f"Marker with ID {marker_id} not found.")
        logger.debug(f"Marker found: {marker}")
        return marker

    def _turn_to_marker(self, target_marker_id: int) -> None:
        """
        Re-orients the robot to face towards a specified marker.

        This method is not needed for the virtual league and is intentionally left unimplemented.

        Assumes that the robot is standing directly in front of the marker.

        Parameters:
            target_marker_id (int): The ID of the target marker.

        Raises:
            NotImplementedError: Always raised as this method is not required for the virtual league.
        """
        logger.warning("_turn_to_marker is not implemented for the virtual league.")
        raise NotImplementedError(
            "_turn_to_marker is not needed for the virtual league."
        )

    def _align_with_marker(self, marker, target_marker_pos: np.ndarray) -> None:
        """
        Aligns the robot with the specified marker.

        This method is not needed for the virtual league and is intentionally left unimplemented.

        Parameters:
            marker (sr.Marker): The marker object to align with.
            target_marker_pos (np.ndarray): The position of the target marker.

        Raises:
            NotImplementedError: Always raised as this method is not required for the virtual league.
        """
        logger.warning("_align_with_marker is not implemented for the virtual league.")
        raise NotImplementedError(
            "_align_with_marker is not needed for the virtual league."
        )

    def get_marker_pos(self, marker_id: int) -> Tuple[float, float]:
        """
        Calculates the absolute position of a marker based on its relative position to the robot.

        Parameters:
            marker_id (int): The ID of the marker.

        Returns:
            Tuple[float, float]: A tuple containing the (x, y) coordinates of the marker.

        Raises:
            ValueError: If the marker with the specified ID is not found.
        """
        logger.debug(f"Calculating position for marker ID {marker_id}.")
        marker = self._get_marker_by_id(marker_id)
        # TODO: Implement position calculation logic based on marker's relative position
        # Example:
        # x = ...  # Calculate x-coordinate
        # y = ...  # Calculate y-coordinate
        # return (x, y)
        raise NotImplementedError("get_marker_pos method needs to be implemented.")

    def move_to_marker(self, marker_id: int) -> None:
        """
        Moves the robot to a specified marker and orients it appropriately.

        Utilises PID controllers to minimise the distance and angle to the target marker within a set tolerance.

        Parameters:
            marker_id (int): The ID of the target marker.

        Raises:
            NotImplementedError: Indicates that the method's implementation is pending.
        """
        logger.info(f"Moving to marker ID {marker_id}.")
        # TODO: Implement movement logic using PID controllers
        raise NotImplementedError("move_to_marker method needs to be implemented.")

    def get_pos(self, test_markers: Optional[List[sr.Marker]] = None) -> Optional[Tuple[float, float, float]]:
        """
        Estimates the robot's position and orientation.

        If `test_markers` are provided, these marker observations are used instead of the camera input, facilitating testing.

        Parameters:
            test_markers (Optional[List[sr.Marker]]): A list of marker observations to use for position estimation. Defaults to None.

        Returns:
            Optional[Tuple[float, float, float]]: A tuple containing the (x, y) position and orientation (theta) of the robot.
                                                 Returns None if insufficient markers are visible.

        Raises:
            NotImplementedError: Indicates that the method's implementation is pending.
        """
        logger.debug("Estimating robot's position.")
        # TODO: Implement position estimation logic here
        raise NotImplementedError("get_pos method needs to be implemented.")

    def set_robot_orientation(self, theta: float, phi: float) -> None:
        """
        Sets the robot's orientation to a specified angle using PID control.

        The method adjusts the robot's orientation from the current angle `phi` to the target angle `theta`, minimising the error through PID control.

        Parameters:
            theta (float): The target orientation angle in radians.
            phi (float): The current orientation angle in radians.

        Raises:
            NotImplementedError: Indicates that the method's implementation is pending.
        """
        logger.info(f"Setting robot orientation from {phi} to {theta} radians.")
        # TODO: Implement orientation setting logic using PID controller
        raise NotImplementedError(
            "set_robot_orientation method needs to be implemented."
        )

    def move_pos(self, target_position: Tuple[float, float]) -> None:
        """
        Moves the robot to a specific (x, y) position using PID controllers for angle and distance.

        The method calculates the required adjustments in motor power to navigate the robot to the desired position accurately.

        Parameters:
            target_position (Tuple[float, float]): The (x, y) coordinates to move the robot to.

        Raises:
            NotImplementedError: Indicates that the method's implementation is pending.
        """
        logger.info(f"Moving to position {target_position}.")
        # TODO: Implement movement to position using PID controllers
        raise NotImplementedError("move_pos method needs to be implemented.")

    def power(self, wheels: Tuple[float, float], duration: float, off: bool = True,) -> None:
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
        corrected_wheels = self._wonky_wheels_correction(wheels)
        logger.debug(
            f"Original wheel powers: {wheels}. Corrected wheel powers: {corrected_wheels}."
        )
        self._prev_power = corrected_wheels
        self.motor_board.motors[0].power = self._clamp(corrected_wheels[0])
        self.motor_board.motors[1].power = self._clamp(corrected_wheels[1])
        logger.info(
            f"Setting motor powers to {self._prev_power} for {duration} seconds."
        )
        self.sleep(duration)
        if off:
            self.motor_board.motors[0].power = 0.0
            self.motor_board.motors[1].power = 0.0
            self.sleep(0.01)
            logger.info("Motors turned off.")

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
        logger.debug(f"Clamping value {value} to {clamped_value}.")
        return clamped_value

    def _wonky_wheels_correction(self, wheels: Tuple[float, float]) -> Tuple[float, float]:
        """
        Applies correction to the wheel power values.

        This method can be customised to correct any discrepancies in wheel behavior. Currently reserved for physical robot implementations.

        For the virtual league, no correction is applied.

        Parameters:
            wheels (Tuple[float, float]): The raw wheel power values as (left, right).

        Returns:
            Tuple[float, float]: The corrected wheel power values.

        Raises:
            NotImplementedError: Indicates that wheel correction is not implemented for the virtual league.
        """
        logger.warning(
            "_wonky_wheels_correction is not implemented for the virtual league."
        )
        return wheels  # Assuming no correction needed in simulation

    def get_marker_type(self, marker: sr.Marker | int) -> Optional[str]:
        """
        Determines the type of a marker based on its ID.

        Categorises markers into predefined types such as walls, pallets, and highrise markers based on their ID ranges.

        Parameters:
            marker (Union[sr.Marker, int]): The marker object or raw marker ID integer.

        Returns:
            Optional[str]: The type of marker, which can be one of the following:
                - 'wall' (IDs 0-27)
                - 'pallet0' (IDs 100-119)
                - 'pallet1' (IDs 120-139)
                - 'pallet2' (IDs 140-159)
                - 'pallet3' (IDs 160-179)
                - 'highrise' (IDs 195-199)
                - None if the marker ID does not match any known type.

        Raises:
            ValueError: If the provided marker is neither an sr.Marker instance nor an integer.
        """
        if isinstance(marker, sr.Marker):
            marker_id = marker.id
        elif isinstance(marker, int):
            marker_id = marker
        else:
            logger.error(f"Invalid marker type: {type(marker)}")
            raise ValueError(f"Invalid marker type: {type(marker)}")

        logger.debug(f"Determining type for marker ID: {marker_id}")

        match marker_id:
            case m_id if 0 <= m_id <= 27:
                return "wall"
            case m_id if 100 <= m_id <= 119:
                return "pallet0"
            case m_id if 120 <= m_id <= 139:
                return "pallet1"
            case m_id if 140 <= m_id <= 159:
                return "pallet2"
            case m_id if 160 <= m_id <= 179:
                return "pallet3"
            case m_id if 195 <= m_id <= 199:
                return "highrise"
            case _:
                logger.warning(f"Unknown marker ID: {marker_id}")
                return None
