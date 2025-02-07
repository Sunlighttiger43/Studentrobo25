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

    def run(self) -> Optional[State]:
        """
        Executes the main loop logic to calculate intensity and determine the next state.

        Returns:
            Optional[State]: The next state based on intensity, or None to terminate.
        """

        # timeElapsed = self.get_comp_time()
        # if timeElapsed < 110:
        #     return Hunting(self.r)
        # else:
        #     return Knocking(self.r)
        
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
        try:
            if self.checkVacuumEnabled() == False:
                closestPallet = self.get_closest_pallet()
                if closestPallet == None:
                    self.reverse_and_reset()
                    return MainLoop(self.r)
                print(f"Closest marker ID: {closestPallet.id}")
                self.move_to_marker(closestPallet.id)
                self.sleep(0.5)
                self.pick_up_box()
            closestHirise = self.get_closest_hirise()
            if closestHirise == None:
                self.reverse_and_reset()
                return MainLoop(self.r)
            self.move_to_marker(closestHirise.id)
            self.sleep(0.5)
            self.drop_box()
            self.reverse_and_reset()
            return MainLoop(self.r)
        except:
            self.reverse_and_reset()
            return MainLoop(self.r)