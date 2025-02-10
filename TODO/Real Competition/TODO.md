# Code TODOs:
## olaves_robot TODOs:
- In the following movement functions ("_turn_to_marker", "move_to_marker") add handling logic for when the target marker is no longer visible
- In the following movement functions ("_turn_to_marker", "move_to_marker", "_move_to_marker_finalisation", "move_pos"(may be able to use the same controllers as "move_to_marker") tune PIDControllers for the physical robot (it is important to note that you can not use the same tunings between different systems)
- Implement Positioning logic in "get_pos" combining camera and encoder position systems (look into Kalman Filters, if uncertain, contact Kirollos for help with linalg)
- Implement Positioning logic in "camera_pos" using camera utilities to estimate position
- Implement orientation logic in "set_robot_orientation" using positioning logic and PID contol to set the robot's orientation to a desired value
- Adjust "_apply_min_power" to allow for turning on the spot (currently doesn't have the capability to distinguish between a turn and a move)

## robot TODOs:
- Configure logger

## states TODOs:
- Review viability of states logic in real competition
    - Double check with the design team to be more aware of the goal
- In "Hunting.which_marker", add error handling to deal with the issue of no visible markers in our zone
- In "Hunting.run", add logic for knowing where to drop off pallets
- In "Hunting.run", add interrupt handling logic

# Manufacturing TODOs:
- Refer to the des team, I literally have no clue what's going on here.