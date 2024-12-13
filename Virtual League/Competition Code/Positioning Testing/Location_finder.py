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
Wallmarker0 = (-2155.5,2875)
Wallmarker1 = (-1437.5,2875)
Wallmarker2 = (-719.5,2875)
Wallmarker3 = (0,2875)
Wallmarker4 = (719.5,2875)
Wallmarker5 = (1437.5,2875)
Wallmarker6 = (2155.5,2875)
Wallmarker7 = (2875,2155.5)
Wallmarker8 = (2875,1437.5)
Wallmarker9 = (2875,719.5)
Wallmarker10 = (2875,0)
Wallmarker11 = (2875,-719.5)
Wallmarker12 = (2875,-1437.5)
Wallmarker13 = (2875,-2155.5)
Wallmarker14 = (2155.5,-2875)
Wallmarker15 = (1437.5,-2875)
Wallmarker16 = (719.5,-2875)
Wallmarker17 = (0,-2875)
Wallmarker18 = (-719.5,-2875)
Wallmarker19 = (-1437.5,-2875)
Wallmarker20 = (-2155.5,-2875)
Wallmarker21 = (-2875,-2155.5)
Wallmarker22 = (-2875,-1437.5)
Wallmarker23 = (-2875,-719.5)
Wallmarker24 = (-2875,0)
Wallmarker25 = (-2875,719.5)
Wallmarker26 = (-2875,1437.5)
Wallmarker27 = (-2875,2155.5)
