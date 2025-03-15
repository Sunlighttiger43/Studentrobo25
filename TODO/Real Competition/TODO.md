# Code TODOs:
## olaves_robot TODOs:
### Completed
- Adjust "_apply_min_power" to allow for turning on the spot (currently doesn't have the capability to distinguish between a turn and a move) (COMPLETED: KIROLLOS)
- Implement "get_marker_orientation" (COMPLETED, MERGED WITH "get_marker_pos": AARIT)
    - Will also be recommended to create a log file for the system we are using for our positioning logic overall
- Create a set PIDs function that initialises all PIDControllers (some can be merged, such as "_turn_to_marker" and "set_robot_orientation") (COMPLETED: KIROLLOS)

### In Progress/Under Review
- In the following movement functions ("_turn_to_marker", "move_to_marker", "_move_to_marker_finalisation", "move_pos"(may be able to use the same controllers as "move_to_marker"))  tune PIDControllers for the physical robot (it is important to note that you can not use the same tunings between different systems) (IN PROGRESS: KIROLLOS (CURRENTLY TINKERING WITH A TUNING ALGORITHM))
- Implement "align_with_marker" function (COMPLETED - TO BE TESTED: KIROLLOS)
- In "_move_to_marker_finalisation": 
    - Implement some error checking to validate the ultrasound reading (COMPLETED: KIROLLOS)
    - Implement some encoder magic to ensure the robot approaches the marker without turning (will need to utilise alignment stuff from "align_with_marker" func.) (COMPLETED - TO BE TESTED: KIROLLOS)
    - Get the function caught up with the new class level pid system (COMPLETED: KIROLLOS)
- Use PID Control to implement a more accurate turn and move function that doesn't use dead reckoning (Can use same PID Controls as "_turn_to_marker_" and "_move_to_marker_finalisation") (COMPLETED - TO BE TESTED: KIROLLOS)
    - Implement this in "get_marker_by_id" (COMPLETED - TO BE TESTED: KIROLLOS)
- Implement orientation logic in "set_robot_orientation" using positioning logic and PID contol to set the robot's orientation to a desired value (COMPLETED - TO BE TESTED: KIROLLOS)
- Implement "get_marker_pos" (COMPLETED - TO BE TESTED: AARIT)
- In the following movement functions ("_turn_to_marker", "move_to_marker") add handling logic for when the target marker is no longer visible (COMPLETED - TO BE TESTED: KIROLLOS)
  Implement Positioning logic in "camera_pos" using camera utilities to estimate position (COMPLETED AARIT - TO BE TESTED)
  - Will also be recommended to create a log file for the system we are using for our positioning logic overall
  - Need to check if coordinates satisfy teh functions needs

### Untouched
- Implement Positioning logic in "get_pos" combining camera and encoder position systems (look into Kalman Filters, if uncertain, contact Kirollos for help with linalg)
    - Will also be recommended to create a log file for the system we are using for our positioning logic overall
- Fix the "_scale" function to properly scale powers around a selected center
    - Currently implemented but commented out because it was really buggy and not really working
    - To be reviewed by someone who knows what they are doing
- 

## robot TODOs:
- Configure logger (Completed: KIROLLOS)

## states TODOs:
### Completed:

### In Progress/Under Review
[All being done by Pragathi, Kushal, Yash, Dev - TO BE TESTED]
- Review viability of states logic in real competition
- Double check with the design team to be more aware of the goal
- In "Hunting.run", add logic for knowing where to drop off pallets
- In "Hunting.run", add fallback logic for when no markers are seen
### Untouched


# Manufacturing TODOs:
- Refer to the des team, I literally have no clue what's going on here.
- Checking what motors we currently have
  
