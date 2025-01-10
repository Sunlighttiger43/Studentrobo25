from sr.robot3 import *
import math
from olaves_robot import OlavesRobot
robot = Robot()

pi=math.pi()
for i in range(3):
    OlavesRobot._move(1.5)
    OlavesRobot._turn(0.75*pi)
    OlavesRobot._move(1.5*(math.sqrt(2)))
    OlavesRobot._turn(0.75*pi)
    OlavesRobot._move(1.5)
    OlavesRobot._turn(0.5*pi)

