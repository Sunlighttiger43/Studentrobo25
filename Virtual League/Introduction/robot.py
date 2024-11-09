from sr.robot3 import Robot

robot = Robot()
def see():
    markers = robot.camera.see()
    print("I can see", len(markers), "markers:")

    for marker in markers:
        print("Marker #{0} is {1} metres away".format(
            marker.id,
            marker.position.distance / 1000,
        ))
    
robot.motor_board.motors[0].power = 1
robot.motor_board.motors[0].power = 0.1

robot.sleep(2)

robot.motor_board.motors[0].power = 0
robot.motor_board.motors[1].power = 0

robot.motor_board.motors[0].power = 0.5
robot.motor_board.motors[1].power = 0.5
robot.sleep(3)
robot.motor_board.motors[0].power = 0
robot.motor_board.motors[1].power = 0

see()
