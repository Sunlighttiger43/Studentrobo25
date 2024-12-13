import math
from typing import List, Tuple
from sr.robot3 import Robot

robot = Robot()

my_motor_board = robot.motor_board
my_motor_board.motors[0] = 1
my_motor_board.motors[1] = 1

robot.sleep(3)

my_motor_board.motors[0] = 0
my_motor_board.motors[1] = 0

robot.sleep(3)

my_motor_board.motors[0] = -1
my_motor_board.motors[1] = -1

robot.sleep(3)

my_motor_board.motors[0] = 0
my_motor_board.motors[1] = 0
