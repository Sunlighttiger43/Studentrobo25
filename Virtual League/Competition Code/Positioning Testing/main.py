import math
from math import pi
from scipy.optimize import least_squares
from sr.robot3 import Robot

# Zone coordinates dictionary
zone_coords = {
    0: [
        # 0-6
        (5.026, 0.000, -pi / 2),
        (4.308, 0.000, -pi / 2),
        (3.590, 0.000, -pi / 2),
        (2.872, 0.000, -pi / 2),
        (2.154, 0.000, -pi / 2),
        (1.436, 0.000, -pi / 2),
        (0.718, 0.000, -pi / 2),
        # 7-13
        (0.001, 0.718, pi),
        (0.001, 1.436, pi),
        (0.001, 2.154, pi),
        (0.001, 2.872, pi),
        (0.001, 3.590, pi),
        (0.001, 4.308, pi),
        (0.001, 5.026, pi),
        # 14-20
        (0.718, 5.750, pi / 2),
        (1.436, 5.750, pi / 2),
        (2.154, 5.750, pi / 2),
        (2.872, 5.750, pi / 2),
        (3.590, 5.750, pi / 2),
        (4.308, 5.750, pi / 2),
        (5.026, 5.750, pi / 2),
        # 21-27
        (5.750, 5.026, 0),
        (5.750, 4.308, 0),
        (5.750, 3.590, 0),
        (5.750, 2.872, 0),
        (5.750, 2.154, 0),
        (5.750, 1.436, 0),
        (5.750, 0.718, 0),
    ],
    1: [
        # 0-6
        (5.750, 5.026, 0),
        (5.750, 4.308, 0),
        (5.750, 3.590, 0),
        (5.750, 2.872, 0),
        (5.750, 2.154, 0),
        (5.750, 1.436, 0),
        (5.750, 0.718, 0),
        # 7-13
        (5.026, 0.000, -pi / 2),
        (4.308, 0.000, -pi / 2),
        (3.590, 0.000, -pi / 2),
        (2.872, 0.000, -pi / 2),
        (2.154, 0.000, -pi / 2),
        (1.436, 0.000, -pi / 2),
        (0.718, 0.000, -pi / 2),
        # 14-20
        (0.001, 0.718, pi),
        (0.001, 1.436, pi),
        (0.001, 2.154, pi),
        (0.001, 2.872, pi),
        (0.001, 3.590, pi),
        (0.001, 4.308, pi),
        (0.001, 5.026, pi),
        # 21-27
        (0.718, 5.750, pi / 2),
        (1.436, 5.750, pi / 2),
        (2.154, 5.750, pi / 2),
        (2.872, 5.750, pi / 2),
        (3.590, 5.750, pi / 2),
        (4.308, 5.750, pi / 2),
        (5.026, 5.750, pi / 2),
    ],
    2: [
        # 0-6
        (0.718, 5.750, pi / 2),
        (1.436, 5.750, pi / 2),
        (2.154, 5.750, pi / 2),
        (2.872, 5.750, pi / 2),
        (3.590, 5.750, pi / 2),
        (4.308, 5.750, pi / 2),
        (5.026, 5.750, pi / 2),
        # 7-13
        (5.750, 5.026, 0),
        (5.750, 4.308, 0),
        (5.750, 3.590, 0),
        (5.750, 2.872, 0),
        (5.750, 2.154, 0),
        (5.750, 1.436, 0),
        (5.750, 0.718, 0),
        # 14-20
        (5.026, 0.000, -pi / 2),
        (4.308, 0.000, -pi / 2),
        (3.590, 0.000, -pi / 2),
        (2.872, 0.000, -pi / 2),
        (2.154, 0.000, -pi / 2),
        (1.436, 0.000, -pi / 2),
        (0.718, 0.000, -pi / 2),
        # 21-27
        (0.001, 0.718, pi),
        (0.001, 1.436, pi),
        (0.001, 2.154, pi),
        (0.001, 2.872, pi),
        (0.001, 3.590, pi),
        (0.001, 4.308, pi),
        (0.001, 5.026, pi),
    ],
    3: [
        # 0-6
        (0.001, 0.718, pi),
        (0.001, 1.436, pi),
        (0.001, 2.154, pi),
        (0.001, 2.872, pi),
        (0.001, 3.590, pi),
        (0.001, 4.308, pi),
        (0.001, 5.026, pi),
        
        (0.718, 5.750, pi / 2),
        (1.436, 5.750, pi / 2),
        (2.154, 5.750, pi / 2),
        (2.872, 5.750, pi / 2),
        (3.590, 5.750, pi / 2),
        (4.308, 5.750, pi / 2),
        (5.026, 5.750, pi / 2),
        # 14-20
        (5.750, 5.026, 0),
        (5.750, 4.308, 0),
        (5.750, 3.590, 0),
        (5.750, 2.872, 0),
        (5.750, 2.154, 0),
        (5.750, 1.436, 0),
        (5.750, 0.718, 0),
        # 21-27
        (5.026, 0.000, -pi / 2),
        (4.308, 0.000, -pi / 2),
        (3.590, 0.000, -pi / 2),
        (2.872, 0.000, -pi / 2),
        (2.154, 0.000, -pi / 2),
        (1.436, 0.000, -pi / 2),
        (0.718, 0.000, -pi / 2),
    ],
}


class PositioningRobot:
    def __init__(self, zone=0):
        """
        Initialise the robot
        """
        # super().__init__()
        self.zone = zone

    def get_position(self, test_markers=None):
        """
        Estimate the robot's position (rx, ry) and orientation rtheta.
        
        :param test_markers: If provided, use these marker observations instead of camera input.
                             This is useful for testing.

        :return: (rx, ry, rtheta) or None if insufficient markers are visible.
        """
        # Stop movement to avoid blurry images
        # self.stop()

        # Get markers from camera if no markers are provided (useful for testing)
        markers = test_markers if test_markers is not None else self.camera.see()

        # Filter for arena boundary markers (ID 0-27)
        arena_markers = [m for m in markers if 0 <= m.id <= 27]

        # We need at least two markers to solve for position and heading
        if len(arena_markers) < 2:
            return None

        # Set initial guesses for the solver
        initial_guess = [1.0, 1.0, 0.0]  # (rx_guess, ry_guess, rtheta_guess)

        # Run the least squares solver
        result = least_squares(
            fun=lambda params: self._residuals(params, arena_markers),
            x0=initial_guess
        )

        # Extract solution
        rx, ry, rtheta = result.x
        return rx, ry, rtheta

    def _residuals(self, params, markers):
        """
        Compute residuals for the least squares problem.
        
        Equations:
        For each marker, given known (mx, my) and measured distance d and angle alpha:
        mx = rx + d*cos(rtheta + alpha)
        my = ry + d*sin(rtheta + alpha)

        Residuals are the difference between predicted and actual marker positions.
        
        :param params: [rx, ry, rtheta]
        :param markers: list of observed markers
        :return: list of residuals
        """
        rx, ry, rtheta = params
        residuals = []

        for m in markers:
            # Convert distance from mm to m
            d = m.position.distance / 1000.0
            alpha = m.position.horizontal_angle

            # Get known global position of this marker
            # Replace 0 with self.zone
            mx, my, _ = zone_coords[self.zone][m.id]

            # Predicted positions
            pred_mx = rx + d * math.cos(rtheta + alpha)
            pred_my = ry + d * math.sin(rtheta + alpha)

            # Add residuals (difference)
            residuals.append(pred_mx - mx)
            residuals.append(pred_my - my)

        return residuals


# Mock classes to simulate marker observations
class MockPosition:
    def __init__(self, distance_mm, horizontal_angle):
        self.distance = distance_mm
        self.horizontal_angle = horizontal_angle

class MockMarker:
    def __init__(self, marker_id, distance_mm, horizontal_angle):
        self.id = marker_id
        self.position = MockPosition(distance_mm, horizontal_angle)
        self.orientation = None


def simulate_marker_observation(true_rx, true_ry, true_rtheta, mx, my):
    """
    Given the robot's true position (true_rx, true_ry, true_rtheta) and a marker's position (mx, my),
    compute the simulated distance and angle from the robot to the marker.
    """
    dx = mx - true_rx
    dy = my - true_ry
    distance = math.sqrt(dx*dx + dy*dy)  # in meters
    angle = math.atan2(dy, dx) - true_rtheta
    return distance * 1000.0, angle  # return mm and radians


def run_test_scenario(zone, true_rx, true_ry, true_rtheta, marker_ids):
    """
    Test a scenario by placing the robot at a known pose (true_rx, true_ry, true_rtheta),
    simulating observations of given marker_ids, and then attempting to solve for the robot's pose.

    :param zone: Which zone we are using.
    :param true_rx: Robot's true x position in meters.
    :param true_ry: Robot's true y position in meters.
    :param true_rtheta: Robot's true orientation in radians.
    :param marker_ids: A list of marker IDs to simulate.
    """
    test_markers = []
    for m_id in marker_ids:
        mx, my, _ = zone_coords[zone][m_id]
        d_mm, a = simulate_marker_observation(true_rx, true_ry, true_rtheta, mx, my)
        test_markers.append(MockMarker(m_id, d_mm, a))

    robot = PositioningRobot(zone=zone)
    estimated = robot.get_position(test_markers=test_markers)

    print("-----------------------------------------------------")
    print(f"Test scenario: Zone {zone}")
    print("True pose:      (rx={:.3f}, ry={:.3f}, rtheta={:.3f})".format(true_rx, true_ry, true_rtheta))
    print("Markers used:", marker_ids)
    if estimated is not None:
        rx_est, ry_est, rtheta_est = estimated
        print("Estimated pose: (rx={:.3f}, ry={:.3f}, rtheta={:.3f})".format(rx_est, ry_est, rtheta_est))
        print("Error in x: {:.3f} m, y: {:.3f} m, θ: {:.3f} rad".format(rx_est - true_rx, ry_est - true_ry, rtheta_est - true_rtheta))
    else:
        print("Could not estimate pose.")


if __name__ == "__main__":
    # A bunch of test cases:
    run_test_scenario(zone=1, true_rx=1.0, true_ry=0.5, true_rtheta=0.0, marker_ids=[0, 1])

    run_test_scenario(zone=2, true_rx=4.5, true_ry=2.5, true_rtheta=pi/2, marker_ids=[21, 22])

    run_test_scenario(zone=3, true_rx=2.0, true_ry=4.5, true_rtheta=pi, marker_ids=[14, 15])

    run_test_scenario(zone=0, true_rx=2.5, true_ry=2.5, true_rtheta=pi/4, marker_ids=[3, 7])

    run_test_scenario(zone=0, true_rx=3.0, true_ry=1.0, true_rtheta=-pi/2, marker_ids=[0, 14])