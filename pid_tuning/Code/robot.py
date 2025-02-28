# robot.py

import logging
import json
import random
import os

from olaves_robot import OlavesRobot
from pid import PIDController
from time import time

# Tests, comment out all but the function to test
from move_pos import run_pid_test

# from move_to_marker import run_pid_test

# from smart_turn import run_pid_test

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

    def propose_new_gains(self, controller_name, pid_label, top_n=3, shake=0.10):
        """
        Use the top_n best solutions from history to form a baseline via averaging,
        then apply a small random 'shake' to each gain.

        Parameters:
            - controller_name: e.g. 'move_to_marker'
            - pid_label: 'dist' or 'angle'
            - top_n: how many of the best solutions to average
            - shake: e.g. 0.10 => ±10% random
        Returns:
            - new_gains dict { 'p': float, 'i': float, 'd': float }
        """

        c_data = self.data.get(controller_name, {})
        history = c_data.get("history", [])

        # If no history, just do a naive default:
        if not history:
            logger.warning(
                "[propose_new_gains] | No history found, returning fallback."
            )
            return {"p": 0.2, "i": 0.05, "d": 0.1}

        relevant_data = []
        for entry in history:
            if pid_label == "dist":
                gains = entry["gains_dist"]
            else:
                gains = entry["gains_angle"]
            relevant_data.append((gains, entry["score"]))

        relevant_data.sort(key=lambda x: x[1])
        top_entries = relevant_data[:top_n]

        sum_p = sum(e[0]["p"] for e in top_entries)
        sum_i = sum(e[0]["i"] for e in top_entries)
        sum_d = sum(e[0]["d"] for e in top_entries)
        n = len(top_entries)

        baseline_p = sum_p / n
        baseline_i = sum_i / n
        baseline_d = sum_d / n

        def perturb(val):
            factor = 1.0 + random.uniform(-shake, shake)
            new_val = val * factor
            return max(self.min_gain, min(self.max_gain, new_val))

        new_p = perturb(baseline_p)
        new_i = perturb(baseline_i)
        new_d = perturb(baseline_d)

        new_gains = {"p": new_p, "i": new_i, "d": new_d}
        logger.info(
            f"[propose_new_gains] | Baseline from top {n}: p={baseline_p:.3f}, i={baseline_i:.3f}, d={baseline_d:.3f}"
            f" => Proposed p={new_p:.3f}, i={new_i:.3f}, d={new_d:.3f}"
        )
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


def run_robot():
    configure_logging()
    logger.info("Starting Robot & PID Tuning")

    r = OlavesRobot()

    tuner = PIDTuningManager("pid_tuning_history.json")

    new_dist = tuner.propose_new_gains("move_to_marker", "dist", top_n=5, shake=0.75)
    new_angle = tuner.propose_new_gains("move_to_marker", "angle", top_n=5, shake=0.75)

    # Run test
    marker_id = 110
    score = run_pid_test(r, new_dist, new_angle, marker_id=marker_id, runs=3)

    # Store results
    tuner.update_results("move_to_marker", new_dist, new_angle, score)
    logger.info("PID Tuning cycle completed.")


run_robot()
