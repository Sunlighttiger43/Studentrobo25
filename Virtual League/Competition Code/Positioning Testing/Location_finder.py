import math
from typing import List, Tuple
from pid import PIDController
from sr.robot3 import Robot

import math
from heapq import heappush, heappop
markers = robot.camera.see()
def location_finder(markers):

    marker_info = [[]]
    count = 0
    for marker in markers:
        marker_info[count].append(marker.position.distance)
        marker_info[count].append(marker.position.horizontal_angle)
