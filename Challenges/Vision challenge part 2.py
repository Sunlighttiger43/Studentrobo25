from sr.robot3 import *
import math
robot = Robot()
markers = robot.camera.see()

pi = math.pi

while True:
    robot.sleep(1)
    Yaw = marker.orientation.yaw
    if Yaw <= pi/12 and Yaw >= -pi/12:
        robot.kch.leds[LED_B].colour = Colour.RED
    elif Yaw < -pi/12:
        robot.kch.leds[LED_A].colour = Colour.BLUE
    else:   
        robot.kch.leds[LED_C].colour = Colour.BLUE



