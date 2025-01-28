# General Code Implements
- Implement Positioning System Logic to boilerplate code
    - encoder_pos implementation (COMPLETED: KIROLLOS)
    - camera_pos implementation (IN PROGRESS: )
- Implement move_pos()  (COMPLETED: KIROLLOS)
- def pick_up_box() method (COMPLETED: KIROLLOS TO BE TESTED)
- def release_box() method (COMPLETED: KIROLLOS TO BE TESTED)
- implement get_our_markers() to account for the fact our robot could be in any one of the 4 zones (IN PROGRESS: )
- Implement knocking_state() based off of the pseudocode in states.py (maybe Agastya can do this) (IN PROGRESS: )
- Main program loop (IN PROGRESS: )

# Bug Fixing and Logical Errors
- get_marker_pos (IN PROGRESS: )
    Issues:

    Incorrect Assignment:
        markerDistance = self._get_marker_by_id(marker_id) assigns a sr.Marker object instead of distance.

    Angle Ranges:
        robot_theta > 270 and robot_theta < 360 uses degrees, but angles are in radians.

    Incomplete Logic:
        After identifying marker_orientation, there’s no guarantee it's set before use.

    Potential Unbound Variable:
        If no marker matches marker_id, marker_orientation remains undefined.

- pick_box_hold (COMPLETED: KIROLLOS)
    Issues:

    Compares a marker object to an integer (marker.id), which is always False.

- move_to_marker and move_pos (COMPLETED: KIROLLOS)
    Issues:

    No termination condition beyond error tolerances. If errors never reach tolerances, it could loop indefinitely.

# Readability and Maintainability
**Generally, larger methods should be refactored and split up into smaller sections of code for easier maintainability, readability and it just looks nicer, I realise these are all functions I've coded but it's 3am so I'm going to sleep and leave these for after the maths mock**
- encoder_pos
- move_pos
- move_to_marker
**General Optimisations**
- Define all constants at a class level where possible
- Use properties for encapsulation (kinda chargeable since we're on python but it personally irks me)
