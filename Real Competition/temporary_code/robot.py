# robot.py
"""
We will now test the main low level functions for our robot.
This involves move_to_marker, pick_hold_box, drop_box and get_marker_by_id

- First test involves simply searching for the box, and picking it up, just to drop it quickly after
- You can then test it without the box initially in sight
"""


import logging
from olaves_robot import OlavesRobot
from math import pi
r = OlavesRobot()

def configure_logging():
    """
    Configures logging for the entire application.

    This function should be called once at the start of the program
    to ensure consistent logging across all modules.
    """
    # Set the default logging level and format
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )


def run_robot():
    """
    Main entry point for the robot program.

    This function configures logging and then starts the main state machine.

    Initial Settings:
        - Just above the endstops
        - Cage fully open
    """
    a = 0.76923

    configure_logging()
    #Initial strategy for like guaranteed first few points
    r._move(1.8*a)
    r.pick_hold_box()
    r._turn(-pi/2)
    r._move(0.4*a)
    r.drop_box()
    r.pick_hold_box()
    r._turn(-pi/2)
    r._move(0.45*a)
    r._turn(pi/2)
    r._move(1.2*a)
    r.drop_box()

    #Idle animation
    r._turn(-pi/2)
    r._move(1.2*a)
    r._turn(pi/2)
    r._move(0.4)
    r.pick_hold_box()
    r._turn(pi/2)
    r._move(0.4)
    r.drop_box()
    r.pick_hold_box()
    r._move(1.65)
    r._turn(-pi/2)
    time_remaining = 150 - r.get_comp_time()
    Competition_end = False
    while not Competition_end:
        if time_remaining <= 20:
            Competition_end = True

    r.drop_box()
    r._move(-1)


run_robot()
