# robot.py

import logging
from olaves_robot import OlavesRobot
from movement_challenge import main

r = OlavesRobot()


def configure_logging():
    """
    Configures logging for the entire application.

    This function should be called once at the start of the program
    to ensure consistent logging across all modules.
    """
    # Set the default logging level and format
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[logging.StreamHandler(), logging.FileHandler("app.log")],
    )


def run_robot():
    """
    Main entry point for the robot program.

    This function configures logging and then starts the main state machine.
    """

    configure_logging()
    # Call main from movement_challenge
    main()


if __name__ == "__main__":
    run_robot()
