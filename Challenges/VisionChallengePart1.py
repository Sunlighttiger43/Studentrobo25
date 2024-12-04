from sr.robot3 import *
robot = Robot()
while True:
    robot.sleep(0.05)
    markers = robot.camera.see()
    for marker in markers:
        pos = marker.position.distance
        if pos > 1500:
            robot.kch.leds[LED_B].colour = Colour.RED
        elif pos >200 and pos<1500:
            robot.kch.leds[LED_B].colour = Colour.BLUE
        elif pos<200:
            robot.kch.leds[LED_B].colour = Colour.GREEN
