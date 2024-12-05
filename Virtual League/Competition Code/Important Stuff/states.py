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
    #Idea is to get marker distance as well as ID and make judgments based upon it
    def seeing(self):
        markers = robot.camera.see()
        for marker in markers:
            print(marker.id, marker.position.distance/1000)

class Knocking(State):
    pass

class Unused(State):
    #testing purposes only
    def run(self):
        while True:
            self.r.sleep(2)
            print(self.r.get_pos())


def main() -> None:
    r = OlavesRobot()
    print("Running")
    print(r.zone)

    StateMachine(Unused(r)).run()