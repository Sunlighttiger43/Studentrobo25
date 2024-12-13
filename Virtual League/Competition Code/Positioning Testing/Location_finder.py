import math
from typing import List, Tuple
from pid import PIDController
from sr.robot3 import Robot

import math
from heapq import heappush, heappop
markers = robot.camera.see()
def location_finder(markers):

    marker_info = []

    for marker in markers:
       marker_info.append([marker.position.distance, marker.position.horizontal_angle])

    # adds all data about the markers to a list/tuple
    # for each marker find the equation of the lines from the marker to the robot
    # find the coordinates of the intersections of the lines
    # average the coordiantes of the intersections
    # output the coordinates as a tuple
