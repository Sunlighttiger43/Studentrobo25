# states

from typing import Optional, List, Tuple
from abc import ABC, abstractmethod
from olaves_robot import OlavesRobot

import logging
import math

# Creating a logger
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# Creating handlers
console = logging.StreamHandler().setLevel(logging.INFO)
file = logging.FileHandler(filename="states.log").setLevel(logging.DEBUG)

# Adding handlers to logger
logger.addHandler(console)
logger.addHandler(file)


class State(ABC):
    def __init__(self, robot: OlavesRobot):
        self.r = robot


class StateMachine:
    def __init__(self, initial_state: State):
        self.initial_state = initial_state

    def run(self) -> None:
        """
        Starts the state machine execution loop.

        IMPORTANT: This function must be called to start running the first state.
        """
        state = self.initial_state
        while state is not None:
            logger.info(f"Running state: {type(state).__name__}")
            state = state.run()
        logger.info("State machine execution completed.")


class MainLoop(State):
    def __init__(self, robot: OlavesRobot):
        super().__init__(robot)

    def _calculate_intensity(self) -> int:
        """
        Calculates the current intensity based on time elapsed and number of pallets.

        Returns:
            int: The calculated intensity value.

        Notes:
            - Intensity increases over time and with the number of pallets.
            - This function can be extended to include more factors influencing intensity.
        """
        elapsed_time = self.r.get_comp_time()
        number_of_pallets = self._get_number_of_pallets()
        boxes_left = 6 - self._get_number_of_pallets()

        if elapsed_time == 150:
            intensity = 3
        elif (
            (boxes_left == 2 and elapsed_time >= 135)
            or (boxes_left == 1 and elapsed_time >= 140)
            or (boxes_left == 0)
        ):
            intensity = 2
        elif boxes_left >= 2:
            intensity = 1
        """
        we remoddelled the intensity so that it will keep hunting until 2 boxes 
        it will switch to the knocking state if we have 15 seconds left with 2 boxes or 1 box left with 10 seconds or we have 0 boxes left
        if the time_elapsed is past the total round time, it will switch to sleep state
        """

        logger.debug(
            f"Calculated intensity: {intensity} (Elapsed Time: {elapsed_time}, Pallettes: {number_of_pallets})"
        )
        return intensity

    def _get_number_of_pallets(self) -> int:
        """
        Retrieves the current number of our pallets on highrises

        Returns:
            int: The number of pallets detected.

        Notes:
            - This function should be implemented to accurately count the number of pallets.
            - It can utilise sensors or camera inputs to determine pallet count.
        """
        # Example Implementation:
        detected_pallets_highrises = self.r.detect_pallets()[0]
        pallet_count = detected_pallets_highrises
        logger.debug(f"Number of high rise pallets detected: {pallet_count}")
        return pallet_count

    def run(self) -> Optional[State]:
        """
        Executes the main loop logic to calculate intensity and determine the next state.

        Returns:
            Optional[State]: The next state based on intensity, or None to terminate.
        """

        intensity = self._calculate_intensity()
        logger.info(f"Current Intensity: {intensity}")

        if intensity == 1:
            logger.info("Transitioning to Hunting state.")
            return Hunting(self.r)
        elif intensity == 2:
            logger.info("Transitioning to Knocking state.")
            return Knocking(self.r)
        elif intensity == 3:
            logger.info("Intensity threshold reached. Terminating.")
            return None


class Hunting(State):
    def __init__(self, robot: OlavesRobot):
        super().__init__(robot)
        self.hunting_start_time = self.r.get_comp_time()
        self.boxes_retrieved = []

    # Idea is to get marker distance as well as ID and make judgments based upon it
    def get_our_markers(self) -> List[Tuple[int, float]]:
        """
        Retrieves a list of own pallet markers.

        Returns:
            List[Tuple[int, float]]: A list of tuples containing marker IDs and their distances.
        """
        markers = self.r.camera.see()
        our_markers = []
        for marker in markers:
            if marker in self.get_our_markers():
                our_markers.append((marker.id, marker.position.distance))
        logger.debug(f"Our markers: {our_markers}")
        if our_markers == []:
            self.robot._turn(3.14 / 2)
            return self.get_our_markers()
        else:
            return our_markers

    def decide_marker_placement(self):
        self.total_towers = 0
        """
        Calculates where to put the next marker
        assumed that we have all six boxes
 
        Plan:
         - first deposit all 6 boxes into the centre (capacity 2)
    
         - when boxes fall, move boxes to the 2nd area (capacity 3)
         - when boxes fall, move boxes to the closest area (capcity 1)
 
        main logic:
        - keep trace of muber of towers wer have made
          if we see more than the number of towers we have made, a box has fallen
          find the box that has fallen(furthest from area centre) and place in next area
        """
        self.move_pos(0,0)
        self.release_box()
        self.total_towers += 1
        visible_pallets = self.detect_highrise_pallets()
        floor_pallets = visible_pallets[1]

        if self.total_towers() < floor_pallets :
            furthest_marker = []
            our_markers = self.get_our_markers()
            for marker in our_markers:
                if self.get_marker_pos(marker[0])  : 
                    "finish checking if the marker is not in the centre area, removes all in the area"
                    pass
                
                

    def judgement_of_distances(self, our_markers: List[Tuple[int, float]]) -> int:
        """
        Determines the closest marker from the list of own markers.

        Parameters:
            our_markers (List[Tuple[int, float]]): A list of (ID, distance) pairs.

        Returns:
            int: The ID of the closest marker.

        Raises:
            ValueError: If 'our_markers' is empty.
        """

        if len(our_markers) == 0:
            raise ValueError("list of markers is empty!")

        min = (0, 10000000)
        for m in our_markers:
            if m[1] < min[1]:
                min = (m[0], m[1])
        return min[0]
        pass

         

    def which_marker(self, our_markers: List[Tuple[int, float]]):
        """
        Determines the closest marker from the list of own markers, if we haven't already got it

        Parameters:
            our_markers (List[Tuple[int, float]]): A list of (ID, distance) pairs.

        Returns:
            int: The ID of the closest marker.

        Raises:
            ValueError: If 'our_markers' is empty.
        """
        if not our_markers:
            logger.warning(
                "[Hunting.which_marker] | No own markers detected for distance judgement. Attempting fallback logic."
            )
            return None

        sorted_markers = sorted(our_markers, key=lambda x: x[1])
        closest_marker_id = sorted_markers[0][0]
        logger.debug(f"Closest not already obtained marker ID: {closest_marker_id}")
        if closest_marker_id not in self.boxes_retrieved:
            self.boxes_retrieved.append(closest_marker_id)
            return closest_marker_id
        else:
            try:
                our_markers.remove(our_markers[0])
                return self.which_marker(our_markers)
            except:
                self.boxes_retrieved.clear()
                return ValueError

    def get_hunting_time(self) -> float:
        """
        Calculates the time elapsed since the Hunting state was initiated.

        Returns:
            float: Elapsed time in seconds.
        """
        current_time = self.robot.get_comp_time()
        hunting_duration = current_time - self.hunting_start_time
        logger.debug(f"Hunting duration: {hunting_duration} seconds.")
        return hunting_duration

    def run(self) -> Optional[State]:
        """
        Executes the hunting state to manage the robot's own pallets.

        Returns:
            Optional[State]: The next state after completing hunting (will be the mainloop), or None to terminate.
        """

        our_markers = self.get_our_markers()
        closest_marker_id = self.which_marker(our_markers)

        if closest_marker_id == None:
            logger.info(
                f"[Hunting.run] | No marker detected, returning to mainloop as a fallback"
            )
            return MainLoop(self.r)
            # TODO: Might be wise to add some fallback here, like turning and checking for
            # markers on the spot for a full rotation before returning to main loop

        logger.info(f"Moving to closest marker ID: {closest_marker_id}")
        self.robot.move_to_marker(closest_marker_id)
        self.robot.pick_hold_box()
        # TODO: need logic on which area to drop the box
        self.robot.move_pos([0, 0])
        self.robot.drop_box()

        return MainLoop(self.robot)


class Knocking(State):
    def __init__(self, robot: OlavesRobot):
        super().__init__(robot)
        self.knocking_start_time = robot.get_comp_time()

    def enemy_boxes(self):
        """
        Here I am trying to identify which enemy boxes it can see which are on high rises
        :return: list[enemy_boxes]
        """
        print(f"Returning list of enemy markers")
        enemy_markers = []
        markers = self.r.camera.see()
        # Finding which marker are enemy markers - will need to integrate this with competition mode with the robot attribute Robot.zone
        for marker in markers():
            type_of_marker = OlavesRobot.get_marker_type(marker)
            if (
                (type_of_marker != "pallet" + str(self.r.zone))
                and type_of_marker != "wall"
                and type_of_marker != "highrise"
                and marker.orientation.pitch < 0
            ):
                enemy_markers.append(marker)

        # So that we are not perpetually in the knocking state if we can't see any boxes to knock
        if not enemy_markers:
            return None

        return enemy_markers

    def enemy_target(self):
        """
        The actual judgement on which box we are knocking over (ie targeting) - simplest thing being implemented is whichever is closest
        :return: marker_id of box to be knocked over
        """
        enemy_markers = self.enemy_boxes()
        print(f"Returning which box to knock over")
        shortest_distance = enemy_markers[0].position.distance
        marker = enemy_markers[0]
        # Simple method to find the shortest distance
        for i in range(len(enemy_markers)):
            distance = enemy_markers[i].position.distance
            if shortest_distance > distance:
                shortest_distance = enemy_markers[i].position.distance
                marker = enemy_markers[i]

        return marker

    def knocking_over(self):
        """
        actually knocks over box with given marker_id input
        :return: If the knocking of the boxes was successful
        """
        enemy_marker = self.enemy_target()
        marker_pos = self.robot.get_marker_pos(enemy_marker)
        self.robot.move_to_marker(enemy_marker)
        self.robot._move(0.8)
        pass

    def run(self) -> Optional[State]:
        try:
            self.knocking_over()
        except ValueError:
            return MainLoop(self.robot)


def main() -> None:
    """
    Main function to initialise the robot and start the state machine.

    Initialises the OlavesRobot and starts the state machine with the MainLoop state.
    """
    robot = OlavesRobot()
    logger.info("Robot initialised and starting state machine.")
    state_machine = StateMachine(MainLoop(robot))
    state_machine.run()
