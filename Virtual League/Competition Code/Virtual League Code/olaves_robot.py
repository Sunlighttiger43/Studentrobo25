import sr.robot3 as sr
from math import pi, sin, cos, atan2, asin, tan
from typing import List, Tuple, Optional
from utils.zone_coords import zone_coords
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
        STEPPER_ID = "STEPPER_ID_HERE"
        STEPPER_RATE = 9600
        super().__init__(ignored_arduinos=[STEPPER_ID], raw_ports=[(STEPPER_ID, STEPPER_RATE)])
        
        # Initialising positioning information
        self._position: Tuple[float, float, float] = (2.750, 0.5, pi / 2)
        self._prev_power: Tuple[float, float] = (0.0, 0.0)
        self.prev_velocity: np.ndarray = np.array([0.0, 0.0, 0.0])
        self.pk_matrix: np.ndarray = np.array([[0.1, 0.0, 0.0],
                                              [0.0, 0.1, 0.0],
                                              [0.0, 0.0, 0.1]])
        self.postime: float = 0.0
        self.calltime: float = 0.0
        self.encoder_index: int = 1
        self.blindness: bool = False

        # Loading the map and graph data
        try:
            with open("maps/map.pkl", "rb") as f:
                self.arena_map = pickle.load(f)
            with open("maps/graph_adj.pkl", "rb") as f:
                self.map_graph = pickle.load(f)
            with open('maps/wall_marker_coords.pkl', "rb") as f:
                all_coords = pickle.load(f)
                self.coords = all_coords[self.zone]
        except FileNotFoundError as e:
            logger.error(f"Map files not found: {e}")
            raise

        # Initialising constants for the map
        self.resolution: int = len(self.arena_map)
        self.step_size: float = 5.750 / self.resolution
        self.resolution_idx: List[Tuple[int, float]] = [
            (i, self.arena_map[0][i][0]) for i in range(len(self.arena_map))
        ]

        self.init_time: float = time()

    def get_comp_time(self) -> float:
        """Returns the time elapsed since the robot was initialised."""
        return time() - self.init_time

    def sleep(self, duration: float) -> None:
        """
        Pauses execution for a specified duration.

        Acts as time.sleep, but for the simulation, as robot.sleep() does not work in the sim.

        Parameters:
            duration (float): The amount of time to sleep in seconds.
        """
        super().sleep(duration)

    def _turn(self, angle: float) -> None:
        """
        Performs a dead reckoning turn.

        This method should not be used outside the OlavesRobot subclass.

        Parameters:
            angle (float): The angle to turn in radians. Positive for left, negative for right.
        """
        ANGLE_TO_TIME = 14.2  # Multiplier to make the turn more accurate
        if angle > 0:
            # Turn left
            self.power((0.4, -0.4), angle / pi * ANGLE_TO_TIME)
        else:
            # Turn right
            self.power((-0.4, 0.4), abs(angle) / pi * ANGLE_TO_TIME)

    def _move(self, distance: float) -> None:
        """
        Performs a dead reckoning move.

        This method should not be used outside the OlavesRobot subclass.

        Parameters:
            distance (float): The distance to move in meters. Positive to move forward, negative to move backward.
        """
        DIST_TO_TIME = 5
        if distance > 0:
            self.power((0.4, 0.5), distance * DIST_TO_TIME)
        else:
            self.power((-0.4, -0.5), abs(distance) * DIST_TO_TIME)

    def get_marker_orientation(self, marker_id: int) -> float:
        """
        Returns the absolute orientation of a specified marker.

        Assumptions:
            - The origin is the corner where the robot starts.
            - The x-axis direction is 0 radians.

        Parameters:
            marker_id (int): The ID of the marker.

        Returns:
            float: The absolute orientation of the marker in radians.
        """
        marker = self._get_marker_by_id(marker_id)
        marker_coords = self.get_marker_pos(marker_id)
        chi = self._calculate_chi(marker)
        robot_x, robot_y, robot_theta = self.get_pos()
        x = self._calculate_x_coordinate(robot_theta, robot_x, marker_coords[1])
        delta_x = marker_coords[0] - x
        delta_y = marker_coords[1] - robot_y
        distance = np.hypot(delta_x, delta_y)
        if distance == 0:
            logger.warning("Distance between robot and marker is zero.")
            z = 0.0
        else:
            z = asin((delta_y / distance) * sin(marker.position.horizontal_angle))
        marker_orientation = z - chi
        return marker_orientation

    def _calculate_chi(self, marker) -> float:
        """
        Calculates the chi angle for marker orientation calculation.

        Parameters:
            marker: The marker object.

        Returns:
            float: The chi angle in radians.
        """
        marker_yaw = marker.orientation.yaw  # In radians
        marker_horizontal_angle = marker.position.horizontal_angle  # In radians
        chi = pi - (marker_yaw + marker_horizontal_angle)
        return chi

    def _calculate_x_coordinate(self, robot_theta: float, robot_x: float, marker_y: float) -> float:
        """
        Calculates the x-coordinate for marker orientation calculation.

        Parameters:
            robot_theta (float): The robot's orientation angle.
            robot_x (float): The robot's x-coordinate.
            marker_y (float): The marker's y-coordinate.

        Returns:
            float: The calculated x-coordinate.
        """
        if not (abs(robot_theta - pi/2) < 1e-5 or abs(robot_theta + pi/2) < 1e-5):
            x = (marker_y / tan(robot_theta)) + robot_x
        else:
            x = robot_x
        return x

    def _get_marker_by_id(self, marker_id: int):
        """
        Retrieves a marker by its ID.

        Parameters:
            marker_id (int): The ID of the marker.

        Returns:
            The marker object.

        Raises:
            ValueError: If the marker is not found.
        """
        marker = next((m for m in self.camera.see() if m.id == marker_id), None)
        if marker is None:
            logger.error(f"Marker with ID {marker_id} not found.")
            raise ValueError(f"Marker with ID {marker_id} not found.")
        return marker

    def _turn_to_marker(self, target_marker_id: int) -> None:
        """
        Re-orients the robot to face towards a specified marker.

        Assumes that the robot is standing directly in front of the marker.

        Parameters:
            target_marker_id (int): The ID of the target marker.
        """
        target_marker_pos = np.array(self.get_marker_pos(target_marker_id))
        marker = self._get_marker_by_id(target_marker_id)
        self._align_with_marker(marker, target_marker_pos)

    def _align_with_marker(self, marker, target_marker_pos: np.ndarray) -> None:
        """
        Aligns the robot with the specified marker.

        Parameters:
            marker: The marker object.
            target_marker_pos (np.ndarray): The position of the target marker.
        """
        first_pass = False
        theta = marker.position.horizontal_angle
        phi = marker.orientation.yaw
        angle_pid = PIDController(setpoint=0, kp=0.3, ki=0, kd=0.1)

        while (7 * pi / 36 <= phi <= 11 * pi / 36) or not first_pass:
            first_pass = True
            angle_tolerance = 0.05  # radians

            while abs(theta) > angle_tolerance:
                angle_correction = -angle_pid.get_correction(theta)
                ANGLE_TO_TIME = 0.1
                if angle_correction > 0:
                    # Turn left
                    self.power((0.4, -0.4), angle_correction / pi * ANGLE_TO_TIME)
                else:
                    # Turn right
                    self.power((-0.4, 0.4), abs(angle_correction) / pi * ANGLE_TO_TIME)

                current_marker = self._get_marker_by_id(marker.id)
                if current_marker:
                    theta = current_marker.position.horizontal_angle
                else:
                    current_position = self.camera_pos()
                    current_coords, phi = current_position[:2], current_position[2]
                    destination_vector = target_marker_pos - current_coords
                    theta = atan2(destination_vector[1], destination_vector[0])

            phi = marker.orientation.yaw
            if 7 * pi / 36 <= phi <= 11 * pi / 36:
                self.power((-0.4, -0.4), 0.5, off=False)
                self.power((0.4, -0.4), 0.2, off=False)


        """
        Returns the marker ID ranges for the current zone.

        Returns:
            Tuple[range, range]: The two ranges of marker IDs for the current zone.
        """
        if self.zone == 0:
            return range(0, 7), range(7, 14)
        elif self.zone == 1:
            return range(7, 14), range(14, 21)
        elif self.zone == 2:
            return range(14, 21), range(21, 28)
        else:
            return range(21, 28), range(0, 7)

    def get_marker_pos(self, marker_id: int) -> Tuple[float, float]:
        """
        Calculates the absolute position of a marker based on its relative position to the robot.

        Parameters:
            marker_id (int): The ID of the marker.

        Returns:
            Tuple[float, float]: The (x, y) position of the marker.
        """
        marker = self._get_marker_by_id(marker_id)
        distance = marker.position.distance / 1000  # Convert to meters
        angle = marker.position.horizontal_angle  # Radians

        robot_x, robot_y, robot_theta = self.get_pos()
        relative_angle = robot_theta - angle
        dx = distance * cos(relative_angle)
        dy = distance * sin(relative_angle)

        marker_pos = (robot_x + dx, robot_y + dy)
        return marker_pos

    def move_to_marker(self, marker_id: int) -> None:
        """
        Moves the robot to a specified marker and orients it appropriately.

        Parameters:
            marker_id (int): The ID of the target marker.
        """
        marker_orientation = self.get_marker_orientation(marker_id)
        marker_pos = np.array(self.get_marker_pos(marker_id))
        point_coords = (
            marker_pos[0] + 0.5 * cos(marker_orientation),
            marker_pos[1] + 0.5 * sin(marker_orientation)
        )
        self.move_through_path(point_coords)

        target_orientation = marker_orientation + pi if marker_orientation < 0 else marker_orientation - pi
        self.set_robot_orientation(theta=target_orientation, phi=self.get_pos()[2])

    def get_pos(self, test_markers=None):
        """
        Estimate the robot's position (rx, ry) and orientation rtheta.
        
        :param test_markers: If provided, use these marker observations instead of camera input.
                             This is useful for testing.

        :return: (rx, ry, rtheta) or None if insufficient markers are visible.
        """
        # Stop movement to avoid blurry images
        # self.stop()

        # Get markers from camera if no markers are provided (useful for testing)
        markers = test_markers if test_markers is not None else self.camera.see()

        # Filter for arena boundary markers (ID 0-27)
        arena_markers = [m for m in markers if 0 <= m.id <= 27]

        # We need at least two markers to solve for position and heading
        if len(arena_markers) < 2:
            return None

        # Set initial guesses for the solver
        initial_guess = [1.0, 1.0, 0.0]  # (rx_guess, ry_guess, rtheta_guess)

        # Run the least squares solver
        result = least_squares(
            fun=lambda params: self._residuals(params, arena_markers),
            x0=initial_guess
        )

        # Extract solution
        rx, ry, rtheta = result.x
        return rx, ry, rtheta

    def _residuals(self, params, markers):
        """
        Compute residuals for the least squares problem.
        
        Equations:
        For each marker, given known (mx, my) and measured distance d and angle alpha:
        mx = rx + d*cos(rtheta + alpha)
        my = ry + d*sin(rtheta + alpha)

        Residuals are the difference between predicted and actual marker positions.
        
        :param params: [rx, ry, rtheta]
        :param markers: list of observed markers
        :return: list of residuals
        """
        rx, ry, rtheta = params
        residuals = []

        for m in markers:
            # Convert distance from mm to m
            d = m.position.distance / 1000.0
            alpha = m.position.horizontal_angle

            # Get known global position of this marker
            # Replace 0 with self.zone
            mx, my, _ = zone_coords[self.zone][m.id]

            # Predicted positions
            pred_mx = rx + d * cos(rtheta + alpha)
            pred_my = ry + d * sin(rtheta + alpha)

            # Add residuals (difference)
            residuals.append(pred_mx - mx)
            residuals.append(pred_my - my)

        return residuals
        """
        Retrieves the current position of the robot.

        Returns:
            Tuple[float, float, float]: The (x, y, theta) position of the robot.
        """
        return self.camera_pos()

    def set_robot_orientation(self, theta: float, phi: float) -> None:
        """
        Sets the robot's orientation to a specified angle.

        Parameters:
            theta (float): The target orientation angle in radians.
            phi (float): The current orientation angle in radians.
        """
        angle_tolerance = 0.1  # radians
        angle_pid = PIDController(setpoint=theta, kp=1.5, ki=0, kd=0.75)

        while abs(theta - phi) > angle_tolerance:
            angle_correction = -angle_pid.get_correction(phi)
            ANGLE_TO_TIME = 0.1
            if angle_correction > 0:
                # Turn left
                self.power((0.4, -0.4), angle_correction / pi * ANGLE_TO_TIME)
            else:
                # Turn right
                self.power((-0.4, 0.4), abs(angle_correction) / pi * ANGLE_TO_TIME)

            phi = self.get_pos()[2]

    def move_through_path(self, target_position: Tuple[float, float], tolerance: int = 0) -> None:
        """
        Navigates the robot through a calculated path to the target position.

        Parameters:
            target_position (Tuple[float, float]): The (x, y) coordinates of the target position.
            tolerance (int): Number of waypoints to ignore from the end of the path.
        """
        try:
            path = self._calculate_path(target_position)
            self._execute_path(path, target_position, tolerance)
        except ValueError as e:
            logger.error(f"ValueError in move_through_path: {e}")
            self.move_pos(target_position)
        except Exception as e:
            logger.error(f"Unexpected error in move_through_path: {e}")
            self.move_pos(target_position)

    def _calculate_path(self, target_position: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Calculates the path from the current position to the target position.

        Parameters:
            target_position (Tuple[float, float]): The (x, y) coordinates of the target position.

        Returns:
            List[Tuple[float, float]]: The calculated path as a list of waypoints.
        """
        start_pos = np.array(self.camera_pos())

        start_x_coord_id = min(
            self.resolution_idx, key=lambda a: abs(a[1] - start_pos[0])
        )[0]
        start_y_coord_id = min(
            self.resolution_idx, key=lambda a: abs(a[1] - start_pos[1])
        )[0]

        target_x_coord_id = min(
            self.resolution_idx, key=lambda a: abs(a[1] - target_position[0])
        )[0]
        target_y_coord_id = min(
            self.resolution_idx, key=lambda a: abs(a[1] - target_position[1])
        )[0]

        start_coordinates = (
            self.resolution_idx[start_x_coord_id][1],
            self.resolution_idx[start_y_coord_id][1],
        )
        target_coordinates = (
            self.resolution_idx[target_x_coord_id][1],
            self.resolution_idx[target_y_coord_id][1],
        )

        logger.info(f"Start Coordinates: {start_coordinates}, Target Coordinates: {target_coordinates}")

        path, _ = self._pathfinding(graph=self.map_graph, start=start_coordinates, end=target_coordinates)
        path.append(target_position)

        logger.debug(f"Calculated Path: {path}")
        return path

    def _pathfinding(self, graph, start, end):
        queue = []
        heapq.heappush(queue, (0, start))
        
        costs = {node: float('inf') for node in graph}
        costs[start] = 0
        
        previous = {node: None for node in graph}
        
        while queue:
            current_cost, current_node = heapq.heappop(queue)
            
            if current_node == end:
                break
            
            for neighbor, weight in graph[current_node]:
                path_cost = current_cost + weight
                if path_cost < costs[neighbor]:
                    costs[neighbor] = path_cost
                    previous[neighbor] = current_node
                    heapq.heappush(queue, (path_cost, neighbor))
        
        path = []
        node = end
        if previous[node] is not None or node == start:
            while node is not None:
                path.insert(0, node)
                node = previous[node]
        return path, costs[end]

    def _execute_path(self, path: List[Tuple[float, float]], target_position: Tuple[float, float], tolerance: int) -> None:
        """
        Executes the movement along the calculated path.

        Parameters:
            path (List[Tuple[float, float]]): The calculated path as a list of waypoints.
            target_position (Tuple[float, float]): The (x, y) coordinates of the target position.
            tolerance (int): Number of waypoints to ignore from the end of the path.
        """
        # Remove waypoints based on tolerance
        for _ in range(tolerance):
            if path:
                path.pop()
        
        for point in path:
            logger.debug(f"Moving to point: {point}")
            self.move_pos(point)

        if tolerance > 0 and path:
            logger.debug(f"Final move to target position: {target_position}")
            self.move_pos(target_position)

    def move_pos(self, target_position: Tuple[float, float]) -> None:
        """
        Moves the robot to a specific position using PID controllers for angle and distance.

        Parameters:
            target_position (Tuple[float, float]): The (x, y) coordinates to move to.
        """
        logger.info(f"Moving to position: {target_position}")

        while True:
            current_position = np.array(self.get_pos())
            distance, theta = self._calculate_distance_and_angle(current_position, target_position)

            if distance <= 0.2:
                logger.info("Reached the target position.")
                break

            left_power, right_power = self._calculate_movement_corrections(distance, theta, current_position[2])

            self._apply_motor_powers(left_power, right_power, distance)

    def _calculate_distance_and_angle(self, current_position: np.ndarray, target_position: Tuple[float, float]) -> Tuple[float, float]:
        """
        Calculates the distance and angle to the target position.

        Parameters:
            current_position (np.ndarray): The current position of the robot.
            target_position (Tuple[float, float]): The target position.

        Returns:
            Tuple[float, float]: The distance and angle to the target.
        """
        destination_vector = np.array(target_position) - current_position[:2]
        distance = np.linalg.norm(destination_vector)
        theta = atan2(destination_vector[1], destination_vector[0])
        return distance, theta

    def _calculate_movement_corrections(self, distance: float, theta: float, current_theta: float) -> Tuple[float, float]:
        """
        Calculates the motor power corrections using PID controllers.

        Parameters:
            distance (float): The distance to the target.
            theta (float): The angle to the target.
            current_theta (float): The current orientation of the robot.

        Returns:
            Tuple[float, float]: The left and right motor power corrections.
        """
        angle_pid = PIDController(setpoint=theta, kp=1.5, ki=0.0, kd=0.75)
        distance_pid = PIDController(setpoint=0.0, kp=0.5, ki=0.2, kd=0.5)

        angle_correction = -angle_pid.get_correction(current_theta)
        distance_correction = -distance_pid.get_correction(distance) * 0.2

        left_power = distance_correction - angle_correction
        right_power = distance_correction + angle_correction

        # Clamp motor power values to [-1.0, 1.0]
        left_power = self._clamp(left_power * 2)
        right_power = self._clamp(right_power * 2)

        logger.debug(f"Motor Powers - Left: {left_power}, Right: {right_power}")
        return left_power, right_power

    def _apply_motor_powers(self, left_power: float, right_power: float, distance: float) -> None:
        """
        Applies motor powers to move the robot.

        Parameters:
            left_power (float): The power for the left motor.
            right_power (float): The power for the right motor.
            distance (float): The distance to the target.
        """
        distance_correction = abs(distance) * 0.1

        if distance > 0:
            self.power((0.4 * left_power, 0.4 * right_power), distance_correction, off=False)
        else:
            self.power((-0.4 * left_power, -0.4 * right_power), distance_correction, off=False)

    def power(self, wheels: Tuple[float, float], duration: float, off: bool = True) -> None:
        """
        Controls the motors by setting their power levels.

        Parameters:
            wheels (Tuple[float, float]): 
                - First element controls the left wheel's power.
                - Second element controls the right wheel's power.
                Positive values move the wheel forward, negative values move it backward.
            duration (float): The duration in seconds for which the motors should run.
            off (bool): Whether to turn off the motors after the duration. Defaults to True.
        """
        wheels = self._wonky_wheels_correction(wheels)
        self._prev_power = wheels
        self.motor_board.motors[0].power = self._clamp(wheels[0])
        self.motor_board.motors[1].power = self._clamp(wheels[1])
        self.sleep(duration)
        if off:
            self.motor_board.motors[0].power = 0.0
            self.motor_board.motors[1].power = 0.0
            self.sleep(0.01)

    @staticmethod
    def _clamp(value: float, min_value: float = -1.0, max_value: float = 1.0) -> float:
        """
        Clamps a value to the specified range.

        Parameters:
            value (float): The value to clamp.
            min_value (float): The minimum allowable value. Defaults to -1.0.
            max_value (float): The maximum allowable value. Defaults to 1.0.

        Returns:
            float: The clamped value.
        """
        return max(min_value, min(max_value, value))

    def _wonky_wheels_correction(self, wheels: Tuple[float, float]) -> Tuple[float, float]:
        """
        Applies correction to the wheel power values.

        This method can be customized to correct any discrepancies in wheel behavior.

        Parameters:
            wheels (Tuple[float, float]): The raw wheel power values.

        Returns:
            Tuple[float, float]: The corrected wheel power values.
        """
        # Placeholder for any correction logic
        return wheels

    def getMarkerType(self, marker: object|int) -> str:
        """
        Takes in a marker object or ID and returns what type of marker it is

        Parameters:
            marker (object|int): The marker object or raw marker ID integer

        Returns
            String: The type of marker it is from:
                - wall: 0-27
                - pallet0: 100-119
                - pallet1: 120-139
                - pallet2: 140-159
                - pallet3: 160-179
                - highrise: 195-199
                (None otherwise)
        """
        if type(marker) == object:
            id = marker.id
        else:
            id = marker
        match id:
            case id if id >= 0 and id <= 27:
                return 'wall'
            case id if id >= 100 and id <= 119:
                return 'pallet0'
            case id if id >= 120 and id <= 139:
                return 'pallet1'
            case id if id >= 140 and id <= 159:
                return 'pallet2'
            case id if id >= 160 and id <= 179:
                return 'pallet3'
            case id if id >= 195 and id <= 199:
                return 'highrise'
            case _:
                return None
