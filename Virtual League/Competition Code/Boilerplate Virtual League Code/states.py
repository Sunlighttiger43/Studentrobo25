#robot states

from abc import ABC, abstractmethod
import numpy as np
from olaves_robot import OlavesRobot
from math import pi, sin, cos, floor, atan, tan
my_power_board=OlavesRobot.power_board

class State(ABC):
    def __init__(self, robot: OlavesRobot):
        self.r = robot


class StateMachine:
    def __init__(self, initialState: State):
        self.initialState = initialState

    def run(self):
        '''IMPORTANT: This function must be called to start running the first state'''
        state = self.initialState
        while state != None:
            print("Running:", type(state).__name__)
            state = state.run()
        print("Ran All")


class MainLoop(State):
    def __init__(self, robot: OlavesRobot):
        super().__init__(robot)
        self.time = robot.get_comp_time()

    def calculate_intensity(self):
        '''Calculates the current intensity based on time elapsed and number of pallets
        
        This function should not be called outside of main_loop() other than for testing purposes
        
        Parameters:
            None
        Returns:
            float
        '''
        intensity = int(self.get_comp_time()/10)
        # TODO: Factor in number of pallets into the calculation
        return intensity
    
    def get_number_of_pallets(self):
        '''Gets the number of pallets [insert condition]
        
        This function should not be called outside calculate_intensity() other than for testing purposes
        Parameters:
            None
        Returns:
            int'''
        pass
        # TODO: Iimplement this function to return a meaningful number to be used in the intensity calculations
        # Figure out what exactly this function should be returning
    
    def main_loop(self):
        '''IMPORTANT: Calculates intensity and decides which state to call based on the intensity

        Parameters:
            None
        Returns:
            None 
        '''
        intensity = self.calculate_intensity()
        if intensity < 10 :
            Hunting() 
        elif intensity < 15 :
            Knocking()
        else:
            pass
        # TODO: Modify the decision statements to reflect any changes in the intensity calculation
        # Add any other states if they are needed


class Hunting(State):
    def __init__(self, robot: OlavesRobot):
        super().__init__(robot)
        self.hunting_start_time = robot.get_comp_time()
    
    #Idea is to get marker distance as well as ID and make judgments based upon it
    def get_our_markers(self):
        '''Gets a list of our zone pallet markers
        
        Parameters: 
            None
        Returns:
            List[int, float]
        '''
        markers = self.r.camera.see()
        our_markers = []
        for marker in markers:
            if marker.id >= 100 and marker.id<= 119:
            # MARKER ID RANGE DEPENDENT ON ROBOT ZONE
                our_markers.append([marker.id, marker.position.distance])
        # TODO: Change the implementation to account for the fact that our robot could be in any of the 4 zones
        return our_markers

    def judgement_of_distances(self, our_markers):
        '''Returns the shortest distance marker from a list of marker id-distance pairs

        Routinely call this method whenever the robot moves to ensure shortest-distance pallet stays updated
        
        Parameters:
            our_markers (List[List[int, float]]): 
                A list of [id, position] pairs. 
                Should be generated using the get_our_markers() function.
        Returns:
            int
        Raises:
            ValueError: If 'our_markers' is empty.
        '''
        SortedMarkers = sorted(our_markers.items(), key=lambda x:x[1])
        return SortedMarkers[0][0]

    def return_to_origin(self):
        '''Moves robot to origin on coordinate system and turns to a default angle

        Parameters:
            None
        Returns:
            None 
        '''
        self.r.move_through_path([0, 0])
        # TODO: Turn robot to a default angle (using self.r.turn)

    def get_hunting_time(self):
        '''Returns the time elapsed since thhis instance of the Hunting state began
        
        Parameters:
            None
        Returns:
            float
        '''
        self.new_time = self.r.get_comp_time()
        return self.new_time - self.hunting_start_time


class Knocking(State):
    def __init__(self, robot: OlavesRobot):
        super().__init__(robot)
        self.knocking_start_time = robot.get_comp_time()


def main() -> None:
    r = OlavesRobot()
    print("Running")
    print(r.zone)

    StateMachine(Unused(r)).run()

'''SKELETON CODE (blueprint) FOR KNOCKING STATE

the robot has to find opponent boxes to push over
this can be in a systematic way: 1. searching for opponent box stacks, from which the top box doesn't have the id of the robots own boxes 
2. knocking them over

PSEUDOCODE FOR SEARCHING ALGORITHM:
Define robot_own_ids as a list of IDs belonging to the robot
Define detected_boxes as an empty list

Function scan_for_boxes():
    Use sensors to detect all boxes in the arena
    Return a list of boxes with their positions and IDs

Function move_to_position(position):
    Navigate the robot to the specified position

Function knock_over_stack(position):
    Perform the action to knock over a stack at the given position

While True:
    detected_boxes = scan_for_boxes()

    If detected_boxes is empty:
        Search the arena systematically (e.g., follow a grid pattern)

    For each box in detected_boxes:
        If top box ID is NOT in robot_own_ids:
            move_to_position(box.position)
            knock_over_stack(box.position)
            Break out of loop
    End For
End While

CODE ITSELF:

# List of IDs belonging to the robot
robot_own_ids = [101, 102, 103]  # Example IDs for the robot's own boxes

def scan_for_boxes():
    """
    Simulate scanning for boxes in the arena.
    Returns:
        A list of detected boxes, where each box is represented as a dictionary 
        with its position (x, y) and ID.
    Example Output:
        [{"position": (5, 10), "id": 201}, {"position": (10, 15), "id": 102}]
    """
    # Replace this simulated output with actual sensor integration
    return [{"position": (5, 10), "id": 201}, {"position": (10, 15), "id": 102}]

def move_to_position(position):
    """
    Move the robot to the specified position.
    Args:
        position (tuple): The (x, y) coordinates to move to.
    """
    print(f"Moving to position {position}")
    # Replace this print statement with actual navigation code

def knock_over_stack(position):
    """
    Knock over the stack of boxes at the specified position.
    Args:
        position (tuple): The (x, y) coordinates of the stack to knock over.
    """
    print(f"Knocking over stack at {position}")
    # Replace this print statement with the actual knocking mechanism

def systematic_search(arena_width, arena_height, step_size):
    """
    Generate a systematic search path in a zigzag pattern to cover the entire arena.
    Args:
        arena_width (int): The width of the arena in units.
        arena_height (int): The height of the arena in units.
        step_size (int): The step size for each move in units.
    Returns:
        List of (x, y) positions defining the search path.
    """
    search_path = []  # List to store the generated search positions
    y = 0  # Start at the bottom of the arena
    direction = 1  # 1 means moving right, -1 means moving left

    while y < arena_height:
        if direction == 1:
            # Moving right across the arena
            for x in range(0, arena_width, step_size):
                search_path.append((x, y))
        else:
            # Moving left across the arena
            for x in range(arena_width - 1, -1, -step_size):
                search_path.append((x, y))

        # Move up to the next row
        y += step_size
        # Switch direction for zigzag pattern
        direction *= -1

    return search_path

def main():
    """
    Main function to coordinate the robot's box searching and knocking actions.
    Includes both box detection and systematic search.
    """
    # Define arena dimensions and step size for systematic search
    arena_width = 20  # Width of the arena in units
    arena_height = 10  # Height of the arena in units
    step_size = 2  # Step size for systematic search in units

    # Generate the systematic search path
    search_path = systematic_search(arena_width, arena_height, step_size)
    current_position_index = 0  # Keeps track of the robot's progress in the search path

    while True:
        # Step 1: Scan for boxes in the arena
        detected_boxes = scan_for_boxes()

        if detected_boxes:  # If any boxes are detected
            for box in detected_boxes:
                box_id = box["id"]  # Get the ID of the detected box
                position = box["position"]  # Get the position of the detected box

                # Check if the top box's ID does not belong to the robot
                if box_id not in robot_own_ids:
                    print(f"Found box with ID {box_id} at {position} that is not ours.")
                    # Move to the box's position and knock it over
                    move_to_position(position)
                    knock_over_stack(position)
                    break  # Exit the loop after knocking over one stack
        else:  # If no boxes are detected
            if current_position_index < len(search_path):  # If the search path is not complete
                # Get the next position in the search path
                next_position = search_path[current_position_index]
                print(f"No boxes detected. Moving to systematic search position {next_position}")
                # Move to the next position in the systematic search
                move_to_position(next_position)
                # Increment the position index to track progress
                current_position_index += 1
            else:  # If the search path is complete
                print("Systematic search complete. Restarting...")
                # Reset the search path to restart coverage of the arena
                current_position_index = 0

# Run the main function
main()

'''
