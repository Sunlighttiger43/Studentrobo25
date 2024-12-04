#robot states
from abc import ABC, abstractmethod
import numpy as np
from olaves_robot import OlavesRobot
from math import pi, sin, cos, floor, atan, tan

class State(ABC):
    def __init__(self, robot: OlavesRobot):
        self.r = robot


class StateMachine:
    def __init__(self, initialState: State):
        self.initialState = initialState

    def run(self):
        state = self.initialState
        while state != None:
            print("Running:", type(state).__name__)
            state = state.run()
        print("Ran All")

class Hunting(State):
    pass

class Knocking(State):
    pass

class Unused(State):
    #testing purposes only
    pass

def main() -> None:
    r = OlavesRobot()
    print("Running")
    print(r.zone)

    StateMachine(Hunting(r)).run()