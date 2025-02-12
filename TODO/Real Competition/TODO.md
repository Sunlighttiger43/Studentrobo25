# Code TODOs:
## olaves_robot TODOs:
- In the following movement functions ("_turn_to_marker", "move_to_marker") add handling logic for when the target marker is no longer visible (COMPLETED: KIROLLOS)
- In the following movement functions ("_turn_to_marker", "move_to_marker", "_move_to_marker_finalisation", "move_pos"(may be able to use the same controllers as "move_to_marker"))  tune PIDControllers for the physical robot (it is important to note that you can not use the same tunings between different systems) (IN PROGRESS: KIROLLOS (CURRENTLY TINKERING WITH A TUNING ALGORITHM))
- Implement Positioning logic in "get_pos" combining camera and encoder position systems (look into Kalman Filters, if uncertain, contact Kirollos for help with linalg)
    - Will also be recommended to create a log file for the system we are using for our positioning logic overall
- Implement Positioning logic in "camera_pos" using camera utilities to estimate position
    - Will also be recommended to create a log file for the system we are using for our positioning logic overall
- Implement orientation logic in "set_robot_orientation" using positioning logic and PID contol to set the robot's orientation to a desired value
- Adjust "_apply_min_power" to allow for turning on the spot (currently doesn't have the capability to distinguish between a turn and a move) (COMPLETED: KIROLLOS)
- Implement "get_marker_orientation" (TO BE REVIEWED, I THINK AARIT HAS INADVERTENTLY COMPLETED THIS)
    - Will also be recommended to create a log file for the system we are using for our positioning logic overall
- Implement "get_marker_pos" (COMPLETED - TO BE TESTED: AARIT)
- Use PID Control to implement a more accurate turn and move function that doesn't use dead reckoning (Can use same PID Controls as "_turn_to_marker_" and "_move_to_marker_finalisation")
    - Implement this in "get_marker_by_id"
- In "_move_to_marker_finalisation", implement some error checking to validate the ultrasound reading
- In "_move_to_marker_finalisation", implement some encoder magic to ensure the robot approaches the marker without turning (will need to utilise alignment stuff from "align_with_marker" func.)
- Fix the "_scale" function to properly scale powers around a selected center
    - Currently implemented but commented out because it was really buggy and not really working
    - To be reviewed by someone who knows what they are doing
- Implement "align_with_marker" function

## robot TODOs:
- Configure logger (Completed: KIROLLOS)

## states TODOs:
- Review viability of states logic in real competition
    - Double check with the design team to be more aware of the goal
- In "Hunting.run", add logic for knowing where to drop off pallets
- In "Hunting.run", add fallback logic for when no markers are seen

# Manufacturing TODOs:
- Refer to the des team, I literally have no clue what's going on here.