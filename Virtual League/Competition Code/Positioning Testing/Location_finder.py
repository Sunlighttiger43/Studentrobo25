from typing import List, Tuple
from pid import PIDController
from sr.robot3 import Robot
import math
from random import random 

from sympy import Matrix,linsolve,symbols
x, y = symbols("x, y")
from heapq import heappush, heappop
markers = robot.camera.see()

equation_list = []

def equationmaker(distance, xcoord , ycoord, angle):
    m = distance*tan(math.degrees(angle))
    c = ycoord-(m*xcoord)
    Eqn = [-1*m*x + y - c]
    equation_list.append(Eqn)

def equationsolver(eqa, eqb): # fu+nction to find the common coordinate of 2 lines
    x = len(equation_list)
    for i in range(floor(len(x/2))):
        linsolve((eqa, eqb), x, y)    
        

def location_finder(markers):

    marker_info = []

    for marker in markers:
        marker_info.extend ([marker.position.distance, marker.position.horizontal_angle, marker.id, x coord of key(marker.id), y coord of key(marker.id)])

        

    # for each marker find the equation of the lines from the marker to the robot
    
    for i in marker_info:
        equationmaker(i[0],i[3],i[4],i[1])
    for i in range(math.floor(len(equation_list)/2)):
        new_total=0
        new_total=equationsolver(equation_list[i], equation_list[random.choice(equation_list)])
        new_total=new_total/(math.floor(len(equation_list)/2))

    return new_total
    
    
    

    
Marker_Dictionary = {
"Wallmarker0": (-2155.5, 2875),
"Wallmarker1": (-1437.5, 2875),
"Wallmarker2": (-719.5, 2875),
"Wallmarker3": (0, 2875),
"Wallmarker4": (719.5, 2875),
"Wallmarker5": (1437.5, 2875),
"Wallmarker6": (2155.5, 2875),
"Wallmarker7": (2875, 2155.5),
"Wallmarker8": (2875, 1437.5),
"Wallmarker9": (2875, 719.5),
"Wallmarker10": (2875, 0),
"Wallmarker11": (2875, -719.5),
"Wallmarker12": (2875, -1437.5),
"Wallmarker13": (2875, -2155.5),
"Wallmarker14": (2155.5, -2875),
"Wallmarker15": (1437.5, -2875),
"Wallmarker16": (719.5, -2875),
"Wallmarker17": (0, -2875),
"Wallmarker18": (-719.5, -2875),
"Wallmarker19": (-1437.5, -2875),
"Wallmarker20": (-2155.5, -2875),
"Wallmarker21": (-2875, -2155.5),
"Wallmarker22": (-2875, -1437.5),
"Wallmarker23": (-2875, -719.5),
"Wallmarker24": (-2875, 0),
"Wallmarker25": (-2875, 719.5),
"Wallmarker26": (-2875, 1437.5),
"Wallmarker27": (-2875, 2155.5)
}

