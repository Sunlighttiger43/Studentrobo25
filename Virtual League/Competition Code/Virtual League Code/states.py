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
        state = self.initialState
        while state != None:
            print("Running:", type(state).__name__)
            state = state.run()
        print("Ran All")
    def Attacking(self):
        pass

    def Defending(self):
        pass


class Hunting(State):
    #Idea is to get marker distance as well as ID and make judgments based upon it
    def seeing(self):
        markers = robot.camera.see()
        OurMarkers = []
        for marker in markers:
            if marker.id >= 100 and marker.id<= 119:
                OurMarkers.append([marker.id, marker.position.distance])
        return OurMarkers

    def judgement_of_distances(self, OurMarkers):
        #Judges which marker is the closest
        SortedMarkers = sorted(OurMarkers.items(), key=lambda x:x[1])
        return SortedMarkers[0][0]
    #We need to routinely update the shortest list because it'll change when you move to each marker
    def go_to_marker(self,marker):
        OlavesRobot._turn_to_marker(self,marker)

    def _get_hunting_time(self)
        

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

