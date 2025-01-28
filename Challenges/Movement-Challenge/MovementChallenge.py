from sr.robot3 import *
import math
from olaves_robot import OlavesRobot
r = OlavesRobot()

pi=math.pi
for i in range(3):
    r._move(1.5)
    r._turn(0.75*pi)
    r._move(1.5*(math.sqrt(2)))
    r._turn(0.75*pi)
    r._move(1.5)
    r._turn(0.5*pi)

