from sr.robot3 import *
robot = Robot()
while True:
    robot.sleep(0.05)
    markers = robot.camera.see()
    marker = marker[0]
    pos = marker.position.distance
    if pos > 1500:
        robot.kch.leds[LED_B].colour = Colour.RED
    if pos >200 and pos<1500:
        robot.kch.leds[LED_B].colour = Colour.BLUE
    if pos<200:
        robot.kch.leds[LED_B].colour = Colour.GREEN
