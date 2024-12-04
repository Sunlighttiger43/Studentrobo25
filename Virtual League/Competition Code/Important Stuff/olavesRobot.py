import sr.robot3 as sr
from math import pi, sin, cos, sqrt, atan, asin, tan
from typing import List, Tuple
from pid import PIDController
import numpy as np
from time import time
from utils.graph import pathfinding
import pickle
from functools import cmp_to_key


class olavesRobot(sr.Robot):
    ticks = (0, 0)
    def __init__(self):
        print("Initialising")
        STEPPERID = "STEPPER_ID_HERE"
        STEPPER_RATE = 9600
        super().__init__(ignored_arduinos = [STEPPERID], raw_ports = [(STEPPERID, STEPPER_RATE)])
        # Initialising info for positioning
        self._position = (2.750, 0.5, pi / 2)
        self._prev_power = (0.0, 0.0)
        self.prev_v = np.array([0.0, 0.0, 0.0])
        self.pk = np.array([[0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.0, 0.0, 0.1]])
        self.postime = 0.0
        self.calltime = 0.0
        self.encoder_index = 1
        self.blindness = False

        # Loading the map
        with open("maps/map.pkl", "rb") as f:
            self.arena_map = pickle.load(f)
        with open("maps/graph.pkl", "rb") as f:
            self.map_graph = pickle.load(f)
        with open('maps/wall_marker_coords.pkl', "rb") as f:
            self.coords = pickle.load(f)[self.zone]

        # Initialising constants for map
        self.resolution = len(self.arena_map)
        self.step_size = 5.750 / self.resolution
        self.resolution_idx = []
        for i in range(len(self.arena_map)):
            self.resolution_idx.append((i, self.arena_map[0][i][0]))

        self.init_time = time()

    def get_comp_time(self):
        """ Returns time since round started """
        return time() - self.init_time

    def sleep(self, time: float):
        """ Acts as time.sleep, but for the simulation, as robot.sleep() does not work in the sim. """
        super().sleep(time)  # type: ignore

    def _turn(self, angle: float):
        """ Dead Reckoning turn function, not to be used outside olavesRobot subclass """
        ANGLE_TO_TIME = 14.2  # multiplier to make the turn more accurate (not reliable, PIDs to be used when positioing is better)
        if angle > 0:  # turn left
            self.power((0.4, -0.4), angle / pi * ANGLE_TO_TIME)
        else:  # turn right
            self.power((-0.4, 0.4), angle / pi * -ANGLE_TO_TIME)

    def _move(self, distance: float):
        """ Dead Reckoning turn function, not to be used outside olavesRobot subclass """
        DIST_TO_TIME = 5
        if distance > 0:
            self.power((0.4, 0.5), distance * DIST_TO_TIME)
        else:
            self.power((-0.4, -0.5), -distance * DIST_TO_TIME)

    def get_marker_orientation(self, marker_id):
       """ Returns the absolute orientation of a marker. 
            Basic Assumptions:
                - We take the origin to be the corner that our robot starts in. 
                - We take the direction of the x-axis to 0 radians """
       marker = [marker for marker in self.camera.see() if marker.id == marker_id][0]
       marker_yaw = marker.orientation.yaw
       marker_horizontal_angle = marker.position.horizontal_angle
       marker_coords = self.get_marker_pos(marker_id)
       chi = 180 - (marker_yaw + marker_horizontal_angle)
       a, b, phi = self.get_pos()
       y = marker_coords[1]
       x = (y / (tan(phi)) + a) if phi != pi/2 or -pi/2 else a
       d = (marker_coords[0] - x)
       b = (((a - marker_coords[0])**2) + ((b - marker_coords[1])**2))**1/2
       z = asin((b/d) * sin(marker_horizontal_angle))
       marker_orientation = z-chi
       return marker_orientation

    def _turn_to_marker(self, target_marker):
        """ Assuming that the robot is standing directly in fron of a marker, will re-orient the robot to face towards the specified marker """
        target_marker_pos = self.get_marker_pos(target_marker)
        marker = [marker for marker in self.camera.see() if marker.id == target_marker][0]  # Filters markers in camera sight to only show the target marker (filtering to be done before calling the function)
        firstPass = False
        theta = marker.position.horizontal_angle
        phi = marker.orientation.yaw
        angle_pid = PIDController(0, 0.3, 0, 0.1)  # Initialises the PID controller
        while (phi >= 7 * (pi / 36) and phi <= 11 * (pi / 36)) or not firstPass:
            firstPass = True
            angle_tolerance = 0.05  # Initialises the angle tolerance (angle to be fitted witin +- of angle_tolerance)
            while abs(theta) > angle_tolerance:
                angle_correction = -angle_pid.getCorrection(theta)
                ANGLE_TO_TIME = 0.1
                if angle_correction > 0:  # turn left
                    self.power((0.4, -0.4), angle_correction / pi * ANGLE_TO_TIME)
                else:  # turn right
                    self.power((-0.4, 0.4), angle_correction / pi * -ANGLE_TO_TIME)
                if len([marker for marker in self.camera.see() if marker.id == target_marker]) > 0:
                    theta = [marker for marker in self.camera.see() if marker.id == target_marker][0].position.horizontal_angle
                else:
                    current_position = self.camera_pos()
                    current_coords, phi = current_position[:2], current_position[2]
                    destination_vector = target_marker_pos - current_coords
                    theta = atan(destination_vector[1] / destination_vector[0])

            phi = marker.orientation.yaw
            if phi >= 7 * (pi / 36) and phi <= 11 * (pi / 36):
                self.power((-0.4, -0.4), 0.5)
                self.power((0.4, -0.4), 0.2)

    def camera_pos(self):
        """ Returns the robot's position in the map based off of wall markers. Maths is kinda complicated but just think of it as triangulation. """
        # Arena markers that can be seen
        if self.camera.see():
            landmarks = [marker for marker in self.camera.see() if marker.id in range(28)]
            self.blindness = False
        else:
            new_robot_pos = self.encoder_pos()
            self._position = new_robot_pos
            self.blindness = True
            return new_robot_pos #if its true that the encoders arent working and the cameras cant see anything, the position is going to be reliant fully on predictions

        # Cumulative orientation (to get average and find orientation)
        weighted_orientation = 0

        # Checks to prevent corner orientation issue
        range1_satisfied = False
        range2_satisfied = False
        range1_size = 0
        range2_size = 0
        range1_ids = []
        range2_ids = []
        match self.zone:
            case 0:
                for marker in landmarks:
                    if marker.id in range(7):
                        range1_satisfied = True
                        range1_size += 1
                        range1_ids.append(marker.id)
                    if marker.id in range(7, 14):
                        range2_satisfied = True
                        range2_size += 1
                        range2_ids.append(marker.id)
            case 1:
                for marker in landmarks:
                    if marker.id in range(7, 14):
                        range1_satisfied = True
                        range1_size += 1
                        range1_ids.append(marker.id)
                    if marker.id in range(14, 21):
                        range2_satisfied = True
                        range2_size += 1
                        range2_ids.append(marker.id)
            case 2:
                for marker in landmarks:
                    if marker.id in range(14, 21):
                        range1_satisfied = True
                        range1_size += 1
                        range1_ids.append(marker.id)
                    if marker.id in range(21, 28):
                        range2_satisfied = True
                        range2_size += 1
                        range2_ids.append(marker.id)
            case 1:
                for marker in landmarks:
                    if marker.id in range(21, 28):
                        range1_satisfied = True
                        range1_size += 1
                        range1_ids.append(marker.id)
                    if marker.id in range(7):
                        range2_satisfied = True
                        range2_size += 1
                        range2_ids.append(marker.id)

        # Handle wraparound of -π/2 and π at corner
        corner_predicament = range1_satisfied and range2_satisfied
        if corner_predicament:
            print("crisis averted")
            if range1_size > range2_size:
                landmarks = [
                    special_marker
                    for special_marker in landmarks
                    if special_marker.id in range1_ids
                ]
            else:
                landmarks = [
                    special_marker
                    for special_marker in landmarks
                    if special_marker.id in range2_ids
                ]

        # Coordinates for each marker
        landmark_coordinates = np.array([self.coords[marker.id] for marker in landmarks])
        # Angle for each marker
        landmark_yaw = np.array([marker.orientation.yaw * -1 for marker in landmarks])

        # Calculate orientation from each marker and average
        for marker in range(len(landmarks)):
            orientation = landmark_coordinates[marker][2] - landmark_yaw[marker]
            weighted_orientation += orientation

        robot_orientation = (
            weighted_orientation / len(landmarks) if len(landmarks) != 0 else self._position[2]
        )

        # Setting pose variables for markers
        landmark_distance = np.array([marker.position.distance / 1000 for marker in landmarks])
        landmark_hor_angle = np.array([marker.position.horizontal_angle for marker in landmarks])

        # Cumulative position to average
        weighted_pos = np.zeros(2)

        # Adjusting set wall orientations to account for different zones
        wall_orientations: List[range] = [range(0), range(0), range(0), range(0)]
        wall_orientations[0 - self.zone] = range(7)
        wall_orientations[1 - self.zone] = range(7, 14)
        wall_orientations[2 - self.zone] = range(14, 21)
        wall_orientations[3 - self.zone] = range(21, 28)

        for marker in range(len(landmarks)):
            marker_id = landmarks[marker].id
            distance = landmark_distance[marker]
            yaw = landmark_yaw[marker]
            horizontal_angle = -landmark_hor_angle[marker]
            coords = landmark_coordinates[marker][:2]
            # Using vector addition to calculate position based on the coordinates of each wall marker
            if marker_id in wall_orientations[0]:
                pos = np.array(
                    (
                        coords[0] + distance * sin(yaw - horizontal_angle),
                        coords[1] + distance * cos(yaw - horizontal_angle),
                    )
                )
            elif marker_id in wall_orientations[1]:
                pos = np.array(
                    (
                        coords[0] + distance * cos(yaw - horizontal_angle),
                        coords[1] - distance * sin(yaw - horizontal_angle),
                    )
                )
            elif marker_id in wall_orientations[2]:
                pos = np.array(
                    (
                        coords[0] - distance * sin(yaw - horizontal_angle),
                        coords[1] - distance * cos(yaw - horizontal_angle),
                    )
                )
            elif marker_id in wall_orientations[3]:
                pos = np.array(
                    (
                        coords[0] - distance * cos(yaw - horizontal_angle),
                        coords[1] + distance * sin(yaw - horizontal_angle),
                    )
                )
            weighted_pos += pos

        # Calculating the average of all the calculated positions to find an optimal estimate for robot position
        robot_pos = (
            weighted_pos / len(landmarks)
            if len(landmarks) != 0
            else self.encoder_pos()
        )

        new_robot_pos = np.array([robot_pos[0], robot_pos[1], robot_orientation])
        # self._position = new_robot_pos
        return new_robot_pos

    def get_marker_pos(self, marker):
        """ Finds marker position using vectors. """

        # Finds relative distance and angle of marker from camera
        distance, angle = (marker.position.distance/1000, marker.position.horizontal_angle)

        # Calculates vector from camera to marker
        current_x, current_y, theta = self.get_pos()

        relative_angle = theta - angle
        dx, dy = (distance * cos(relative_angle), distance * sin(relative_angle))

        # Calculates marker position
        marker_pos = (current_x + dx, current_y + dy)

        return marker_pos

    def move_to_marker(self, marker_id):
        """ Moves the robot to a specified marker and orients the robot appropriately. """
        marker_orientation = self.get_marker_orientation(marker_id) # get_marker_orientation to be coded by Kirollos (shout at him if he hasn't done it yet)
        point_coords = self.get_marker_pos(marker_id)[0] + 0.5*cos(marker_orientation), self.get_marker_pos(marker_id)[1] + 0.5*sin(marker_orientation)
        self.move_through_path(point_coords)

        target_orientation = marker_orientation + pi if marker_orientation < 0 else marker_orientation-pi
        self.set_robot_orientation(theta=target_orientation, phi=self.get_pos()[2])

    def get_pos(self):
        """ Returns robot position, basically just a decorator as it's an easier function name to remember. """
        return self.camera_pos()

    def set_robot_orientation(self, theta, phi):
        """ Sets the robot orientation to a specified value. """
        angle_tolerance = 0.1
        angle_pid = PIDController(theta, 1.5, 0, 0.75)
        while abs(theta - phi) > angle_tolerance:
            angle_correction = -angle_pid.getCorrection(phi)
            ANGLE_TO_TIME = 0.1
            if angle_correction > 0:  # turn left
                self.power((0.4, -0.4), angle_correction / pi * ANGLE_TO_TIME)
            else:  # turn right
                self.power((-0.4, 0.4), angle_correction / pi * -ANGLE_TO_TIME)

            phi = self.get_pos()[2]

    def move_through_path(self, target_position, tolerance=0):
        try:
            start_pos = self.camera_pos()
            print("hello")  # Koby's contribution

            start_x_coord_id = sorted(
                self.resolution_idx, key=cmp_to_key(lambda a, b: a[1] - start_pos[0])
            )[0][0]
            start_y_coord_id = sorted(
                self.resolution_idx, key=cmp_to_key(lambda a, b: a[1] - start_pos[1])
            )[0][0]

            

            target_x_coord_id = sorted(
                self.resolution_idx, key=cmp_to_key(lambda a, b: a[1] - target_position[0])
            )[0][0]
            target_y_coord_id = sorted(
                self.resolution_idx, key=cmp_to_key(lambda a, b: a[1] - target_position[1])
            )[0][0]

            start_coordinates = (
                self.resolution_idx[start_x_coord_id][1],
                self.resolution_idx[start_y_coord_id][1],
            )
            target_coordinates = (
                self.resolution_idx[target_x_coord_id][1],
                self.resolution_idx[target_y_coord_id][1],
            )

            print(start_coordinates, target_coordinates)

            path = pathfinding(graph=self.map_graph, start=start_coordinates, end=target_coordinates)
            path.append(target_position)
            print(path)
            flag = False
            for i in range(tolerance):
                path.pop()
                flag = True
            for point in path:
                print(point)
                self.move_pos(point)

            if flag:
                self.move_pos(
                    target_position
                )  # Possibly switch to a dedicated move_to_marker func or call a state idk
        except:
            print("uh oh")
            self.move_pos(target_position=target_position)

    def move_pos(self, target_position):
        current_position = self.get_pos()
        current_coords, phi = current_position[:2], current_position[2]
        print("target_pos", type(target_position))
        print("current_pos", type(current_position))
        print("target_pos", (target_position))
        print("current_pos", (current_position))
        destination_vector = target_position - current_coords
        theta = atan(destination_vector[1] / destination_vector[0])
        distance = (destination_vector[0] ** 2 + destination_vector[1] ** 2) ** 0.5
        print("theta", theta)
        print("dist", distance)

        if destination_vector[0] < 0:
            if destination_vector[1] < 0:
                theta -= pi
            else:
                theta += pi

        angle_pid = PIDController(theta, 1.5, 0, 0.75)
        distance_pid = PIDController(0, 0.5, 0.2, 0.5)
        while abs(distance) > 0.2:
            current_position = self.get_pos()
            current_coords, phi = current_position[:2], current_position[2]
            destination_vector = target_position - current_coords
            theta = atan(destination_vector[1] / destination_vector[0])
            distance = (destination_vector[0] ** 2 + destination_vector[1] ** 2) ** 0.5

            if destination_vector[0] < 0:
                if destination_vector[1] < 0:
                    theta -= pi
                else:
                    theta += pi

            distance_correction = -distance_pid.getCorrection(distance) * 0.2
            angle_correction = -angle_pid.getCorrection(phi)

            left = distance_correction - angle_correction
            right = distance_correction + angle_correction

            # Clamps to 1 or -1 if outside bounds
            def clamp(n: float) -> float:
                return n if abs(n) < 1 else 1 * n / abs(n)

            # scales the values to around 0.5, relative to each other
            def shrink(a: float, b: float) -> float:
                return a / (abs(a) + abs(b))

            clampedLeft = clamp(shrink(left, right) * 2)
            clampedRight = clamp(shrink(right, left) * 2)

            print("clamped", clampedLeft, clampedRight)
            DIST_TO_TIME = 0.1
            if distance_correction > 0:
                self.power(
                    (0.4 * clampedLeft, 0.4 * clampedRight),
                    distance_correction * DIST_TO_TIME,
                off = False)
            else:
                self.power(
                    (-0.4 * clampedLeft, -0.4 * clampedRight),
                    -distance_correction * DIST_TO_TIME,
                off = False)

    def power(self, wheels: Tuple[float, float], time: float, off: bool = True):
        """Used to control the motors.

        Parameters:
            wheels (Tuple): The first element controls the left wheel's direction and the second element controls the right wheel's direction. 1 means go forward, 0 means stay still and -1 means go backwards.
            time (float): The amount of time the motors should be on for.
            power (float) = 0.8: The power setting of the motor.
        """
        wheels = self.wonky_wheels_correction(wheels)
        self._prev_power = wheels
        self.motor_board.motors[0].power = wheels[0]
        self.motor_board.motors[1].power = wheels[1]
        self.sleep(time)
        if off:
            self.motor_board.motors[0].power = 0
            self.motor_board.motors[1].power = 0
            self.sleep(0.01)