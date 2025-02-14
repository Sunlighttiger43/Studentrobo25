# move_to_marker.py

import logging

from time import time

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# Creating handlers
console = logging.StreamHandler()
console.setLevel(logging.INFO)
file = logging.FileHandler(filename="move_to_marker_pid.log")
file.setLevel(logging.DEBUG)

# Adding handlers to logger
logger.addHandler(console)
logger.addHandler(file)


def run_pid_test(robot: OlavesRobot, dist_gains, angle_gains, marker_id=110, runs=3):
    """
    Example test routine for 'move_to_marker' that uses the proposed dist & angle gains.
    We measure performance with time to target & final error.

    Real usage:
      1. Set the robot's pid_tunings for 'move_to_marker_dist' and 'move_to_marker_angle'.
      2. Command the robot to do a test (like move_to_marker on a known marker).
      3. Measure performance metrics (time, final error).
      4. Return a "score" (lower is better).

    Parameters:
        - robot: The OlavesRobot instance
        - dist_gains: dict with keys {p, i, d} for distance PID
        - angle_gains: dict with keys {p, i, d} for angle PID
        - marker_id: The ID of the marker to move to
        - runs: How many times we repeat the test to reduce random noise

    Returns:
        - score: float where lower is better.

    """
    robot.move_dist_pid.p = dist_gains["p"]
    robot.move_dist_pid.i = dist_gains["i"]
    robot.move_dist_pid.d = dist_gains["d"]

    robot.move_angle_pid.p = angle_gains["p"]
    robot.move_angle_pid.i = angle_gains["i"]
    robot.move_angle_pid.d = angle_gains["d"]

    total_score = 0.0

    for iteration in range(runs):
        logger.info(
            f"[run_pid_test] | Starting test run {iteration+1}/{runs} with gains: dist={dist_gains}, angle={angle_gains}"
        )

        robot.sleep(5.0)
        start_t = time()

        try:
            robot.move_to_marker(marker_id)
        except Exception as e:
            # If something goes badly wrong, we penalise heavily
            logger.error(f"[run_pid_test] | move_to_marker failed with error: {e}")
            total_score += 999.0
            continue

        end_t = time()
        elapsed = end_t - start_t

        final_marker = robot.get_marker_by_id(marker_id=marker_id)
        if final_marker:
            final_dist = final_marker.position.distance / 1000.0  # in meters
        else:
            # If we lost the marker, treat it as large error
            final_dist = 5.0

        this_run_score = elapsed + 30.0 * final_dist

        logger.info(
            f"[run_pid_test] | Run {iteration+1}/{runs}: elapsed={elapsed:.2f}s, final_dist={final_dist:.2f}m, partial_score={this_run_score:.2f}"
        )
        total_score += this_run_score

    final_score = total_score / runs
    logger.info(f"[run_pid_test] | Final average score: {final_score:.2f}")
    return final_score
