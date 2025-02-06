#Jawad Uddin, 04/02/2025
#states.py
from typing import Optional, List, Tuple
from olaves_robot import OlavesRobot
from abc import ABC, abstractmethod

def main() -> None:
    """
    Main function to initialise the robot and start the state machine.

    Initialises the OlavesRobot and starts the state machine with the MainLoop state.
    """
    robot = OlavesRobot()
    print("Robot initialised and starting state machine.")
    state_machine = StateMachine(MainLoop(robot))
    state_machine.run()

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
            print(f"Running state: {type(state).__name__}")
            state = state.run()
        print("State machine execution completed.")

class MainLoop(State):
    def __init__(self, robot: OlavesRobot):
        super().__init__(robot)

    def calculate_intensity(self):
        return None

    def run(self) -> Optional[State]:
        """
        Executes the main loop logic to calculate intensity and determine the next state.

        Returns:
            Optional[State]: The next state based on intensity, or None to terminate.
        """

        intensity = self.calculate_intensity()
        print(f"Current Intensity: {intensity}")

        return Hunting(self.r)
    


class Hunting(State):
    def __init__(self, robot: OlavesRobot):
        super().__init__(robot)

    def run(self) -> Optional[State]:
        """
        Executes the hunting state to manage the robot's own pallets.

        Returns:
            Optional[State]: The next state after completing hunting (will be the mainloop), or None to terminate.
        """
        closestPallet = self.get_closest_pallet()
        print(f"Closest marker ID: {closestPallet.id}")
        self.sleep(2)


