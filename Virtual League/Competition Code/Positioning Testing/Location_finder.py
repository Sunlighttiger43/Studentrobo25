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
        marker_info.extend ([marker.position.distance, marker.position.horizontal_angle, marker.id, , y coord of key(marker.id)])

        

    # for each marker find the equation of the lines from the marker to the robot
    xcoord_new = 0
    ycoord_new = 0
    for i in marker_info:
        equationmaker(i[0],i[3],i[4],i[1])
    for i in range(math.floor(len(equation_list)/2)):
       eq_output=equationsolver(equation_list[i], equation_list[random.choice(equation_list)])
       xcoord_new += eq_output[0]
       ycoord_new += eq_output[1]
    xcoord_new=xcoord_new/(math.floor(len(equation_list)/2))
    ycoord_new=ycoord_new/(math.floor(len(equation_list)/2))
    return (xcoord_new, ycoord_new )
    
    
    
def angle_finder(xcoord, ycoord):
    angle = math.atan(ycoord/xcoord)
    if angle < 0:
        if xcoord < 0:
            angle = abs(angle) + 270
        elif xcoord >= 0:
            angle = abs(angle) + 90
    elif angle >= 0:
        if xcoord < 0:
            angle = abs(angle) + 180
        else: 
            angle = 90-abs(angle)
    
    return angle
   


    
Marker_Dictionary = {
"0": (-2155.5, 2875),
"1": (-1437.5, 2875),
"2": (-719.5, 2875),
"3": (0, 2875),
"4": (719.5, 2875),
"5": (1437.5, 2875),
"6": (2155.5, 2875),
"7": (2875, 2155.5),
"8": (2875, 1437.5),
"9": (2875, 719.5),
"10": (2875, 0),
"11": (2875, -719.5),
"12": (2875, -1437.5),
"13": (2875, -2155.5),
"14": (2155.5, -2875),
"15": (1437.5, -2875),
"16": (719.5, -2875),
"17": (0, -2875),
"18": (-719.5, -2875),
"19": (-1437.5, -2875),
"20": (-2155.5, -2875),
"21": (-2875, -2155.5),
"22": (-2875, -1437.5),
"23": (-2875, -719.5),
"24": (-2875, 0),
"25": (-2875, 719.5),
"26": (-2875, 1437.5),
"27": (-2875, 2155.5)
}


#why does vsc tell me that there is a colon missing???
