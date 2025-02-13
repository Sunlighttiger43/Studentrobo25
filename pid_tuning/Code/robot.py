# robot.py

import logging
import json
import random
import os
from olaves_robot import OlavesRobot
from pid import PIDController
from time import time

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# Creating handlers
console = logging.StreamHandler()
console.setLevel(logging.INFO)
file = logging.FileHandler(filename="pid.log")
file.setLevel(logging.DEBUG)

# Adding handlers to logger
logger.addHandler(console)
logger.addHandler(file)


class PIDTuningManager:
    def __init__(self, tuning_file="pid_tuning_history.json"):
        self.tuning_file = tuning_file
        self.data = self._load_data()

        self.min_gain = 0.0
        self.max_gain = 2.0

    def _load_data(self):
        if os.path.exists(self.tuning_file):
            with open(self.tuning_file, "r") as f:
                try:
                    d = json.load(f)
                    logger.info("[PIDTuningManager] | Loaded existing tuning data.")
                    return d
                except:
                    logger.warning(
                        "[PIDTuningManager] | Could not parse JSON, using empty."
                    )
                    return {}
        else:
            logger.warning("[PIDTuningManager] | No tuning file found, starting fresh.")
            return {}

    def _save_data(self):
        with open(self.tuning_file, "w") as f:
            json.dump(self.data, f, indent=4)

    def get_best_gains(self, controller_name, pid_label):
        """
        controller_name: e.g. 'move_to_marker'
        pid_label: e.g. 'dist' or 'angle'
        """
        c_data = self.data.get(controller_name, {})
        best_gains_key = f"best_gains_{pid_label}"
        if best_gains_key in c_data:
            return c_data[best_gains_key]
        else:
            # fallback
            return {"p": 0.1, "i": 0.0, "d": 0.0}

    def propose_new_gains(self, best_gains):
        """
        Naive approach: random small perturbation in p, i, d
        """
        new_gains = {}
        for k, v in best_gains.items():
            # random ±10%
            rand_factor = 1.0 + random.uniform(-0.1, 0.1)
            new_gains[k] = v * rand_factor

            candidate = max(self.min_gain, min(self.max_gain, candidate))
            new_gains[k] = candidate
        return new_gains

    def update_results(self, controller_name, dist_gains, angle_gains, score):
        """
        Stores new results; if better, store as best in JSON. Also keep history.
        """
        if controller_name not in self.data:
            self.data[controller_name] = {
                "best_score": float("inf"),
                "best_gains_dist": dist_gains,
                "best_gains_angle": angle_gains,
                "history": [],
            }

        cdata = self.data[controller_name]
        cdata["history"].append(
            {"gains_dist": dist_gains, "gains_angle": angle_gains, "score": score}
        )

        if score < cdata["best_score"]:
            cdata["best_score"] = score
            cdata["best_gains_dist"] = dist_gains
            cdata["best_gains_angle"] = angle_gains

        # Save out
        self._save_data()


def configure_logging():
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    )


def run_pid_test(robot: OlavesRobot, dist_gains, angle_gains, marker_id=0, runs=3):
    """
    Example test routine for 'move_to_marker' that uses the proposed dist & angle gains.
    We measure performance with time to target & final error.

    Real usage:
      1. Set the robot's pid_tunings for 'move_to_marker_dist' and 'move_to_marker_angle'.
      2. Command the robot to do a test (like move_to_marker on a known marker).
      3. Measure performance metrics (time, overshoot, final error).
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

        start_t = time()

        try:
            robot.move_to_marker(marker_id)
        except Exception as e:
            # If something goes badly wrong, we penalise heavily
            logger.error(f"[run_pid_test] | Move to marker failed with error: {e}")
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

        this_run_score = elapsed + 30.0 * final_dist + 10.0

        logger.info(
            f"[run_pid_test] | Run {iteration+1}/{runs}: elapsed={elapsed:.2f}s, final_dist={final_dist:.2f}m, partial_score={this_run_score:.2f}"
        )
        total_score += this_run_score

        robot.sleep(3.0)

    final_score = total_score / runs
    logger.info(f"[run_pid_test] | Final average score: {final_score:.2f}")
    return final_score


def run_robot():
    configure_logging()
    logger.info("Starting Robot & PID Tuning")

    r = OlavesRobot()

    tuner = PIDTuningManager("pid_tuning_history.json")

    best_dist = tuner.get_best_gains("move_to_marker", "dist")
    best_angle = tuner.get_best_gains("move_to_marker", "angle")

    new_dist = tuner.propose_new_gains(best_dist)
    new_angle = tuner.propose_new_gains(best_angle)

    score = run_pid_test(r, new_dist, new_angle)

    tuner.update_results("move_to_marker", new_dist, new_angle, score)

    logger.info("PID Tuning cycle completed.")


run_robot()
