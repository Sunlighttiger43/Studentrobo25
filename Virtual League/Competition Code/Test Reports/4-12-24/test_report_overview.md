# Testers
- Jawad
- Kirollos

# Methods being Tested
- olaves_robot.get_marker_orientation
- olaves_robot.get_pos
- olaves_robot.get_marker_pos
- olaves_robot.move_to_marker
- olaves_robot.set_robot_orientation
- olaves_robot.move_through_path

# Testing Report Overview
- Networkx not in srobo library, utils folder must be removed from robot code
    - Dilemma, we need networkx for pathfinding with robot
    - Kirollos to check with blueshirts whether networkx will be avaliable on the robot


# Testing
## olaves_robot.get_pos
- Was having issues with the code, it was just bugging
- Decided to completely rewrite the code because the maths that was used for that positioning stuff was just not good
- Rewrote positioning code in Positioning Testing
    - Also acts as a place to test positioning logic
    - Will be especially useful when we get to the real robot and implement rotary encoders.