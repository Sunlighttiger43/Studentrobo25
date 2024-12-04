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
- 