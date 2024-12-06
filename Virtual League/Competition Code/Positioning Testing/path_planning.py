import math
from typing import List, Tuple
from pid import PIDController
from sr.robot3 import Robot

import math
from heapq import heappush, heappop


class Planner:
    def __init__(self, obstacles, cell_size=0.05, arena_size=5.75):
        """
        obstacles: list of rectangles (x_min, y_min, x_max, y_max)
        cell_size: size of each cell in meters
        arena_size: total size of the square arena
        """
        self.cell_size = cell_size
        self.arena_size = arena_size
        self.grid_dim = int(arena_size / cell_size)
        
        # Create an occupancy grid
        self.grid = [[False for _ in range(self.grid_dim)] for _ in range(self.grid_dim)]
        
        # Mark obstacles in the grid
        for (ox_min, oy_min, ox_max, oy_max) in obstacles:
            self._mark_obstacle(ox_min, oy_min, ox_max, oy_max)

    def _mark_obstacle(self, ox_min, oy_min, ox_max, oy_max):
        # Convert obstacle coordinates to cell indices
        cx_min = max(0, int(ox_min / self.cell_size))
        cy_min = max(0, int(oy_min / self.cell_size))
        cx_max = min(self.grid_dim - 1, int(ox_max / self.cell_size))
        cy_max = min(self.grid_dim - 1, int(oy_max / self.cell_size))
        
        for x in range(cx_min, cx_max + 1):
            for y in range(cy_min, cy_max + 1):
                self.grid[y][x] = True  # Occupied cell

    def world_to_grid(self, wx, wy):
        """Convert world coordinates to grid coordinates."""
        gx = int(wx / self.cell_size)
        gy = int(wy / self.cell_size)
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Convert grid coordinates to world coordinates (center of the cell)."""
        wx = (gx + 0.5) * self.cell_size
        wy = (gy + 0.5) * self.cell_size
        return wx, wy

    def is_free(self, gx, gy):
        if 0 <= gx < self.grid_dim and 0 <= gy < self.grid_dim:
            return not self.grid[gy][gx]
        return False

    def a_star(self, start_x, start_y, goal_x, goal_y):
        """
        A* search on the grid.
        start_x, start_y, goal_x, goal_y are in world coordinates.
        Returns a list of (wx, wy) waypoints if found, else None.
        """
        start_gx, start_gy = self.world_to_grid(start_x, start_y)
        goal_gx, goal_gy = self.world_to_grid(goal_x, goal_y)

        open_set = []
        heappush(open_set, (0, (start_gx, start_gy)))
        
        came_from = {}
        g_score = {(start_gx, start_gy): 0}
        
        # 8-directional moves or 4-directional
        moves = [(1,0),(-1,0),(0,1),(0,-1),
                 (1,1),(1,-1),(-1,1),(-1,-1)]
        
        def heuristic(gx, gy):
            # Euclidean distance heuristic
            dx = gx - goal_gx
            dy = gy - goal_gy
            return math.sqrt(dx*dx + dy*dy)

        while open_set:
            _, (current_x, current_y) = heappop(open_set)

            # Check goal
            if (current_x, current_y) == (goal_gx, goal_gy):
                # Reconstruct path
                return self._reconstruct_path(came_from, current_x, current_y)

            for mx, my in moves:
                nx, ny = current_x + mx, current_y + my
                if self.is_free(nx, ny):
                    tentative_g = g_score[(current_x, current_y)] + math.sqrt(mx*mx + my*my)
                    if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                        g_score[(nx, ny)] = tentative_g
                        f_score = tentative_g + heuristic(nx, ny)
                        heappush(open_set, (f_score, (nx, ny)))
                        came_from[(nx, ny)] = (current_x, current_y)
        
        # No path found
        return None

    def _reconstruct_path(self, came_from, cx, cy):
        path = [(cx, cy)]
        while (cx, cy) in came_from:
            cx, cy = came_from[(cx, cy)]
            path.append((cx, cy))
        path.reverse()
        # Convert to world coordinates
        world_path = [self.grid_to_world(gx, gy) for gx, gy in path]
        return world_path


class PositioningRobot(Robot):
    def __init__(self, zone=0):
        super().__init__()
        self.zone = zone
        self.linear_pid = PIDController(sp=0, p=0.5, i=0.01, d=0.1)
        self.angular_pid = PIDController(sp=0, p=0.5, i=0.01, d=0.1)
        # Known obstacles
        self.obstacles = [(2.0, 1.5, 2.5, 2.0)]
        self.planner = Planner(self.obstacles, cell_size=0.05, arena_size=5.75)

    def move_to_pose(self, target_x, target_y, target_theta, linear_tolerance=0.05, angular_tolerance=0.05):
        # Compute a path using A*
        pos = self.get_position()
        if pos is None:
            return False
        rx, ry, rtheta = pos

        path = self.planner.a_star(rx, ry, target_x, target_y)
        if path is None:
            print("No path found!")
            return False

        # path is a list of waypoints (x, y)
        # Move along the path
        for (wx, wy) in path:
            # Move to intermediate waypoint
            # Here, we don't necessarily control final orientation at each waypoint,
            # just move directly to waypoint. If desired, align orientation as well.
            success = self._move_to_point(wx, wy, linear_tolerance, angular_tolerance)
            if not success:
                return False

        # Finally, rotate to target_theta at goal
        return self._orient_to_theta(target_theta, angular_tolerance)
    
    def _move_to_point(self, goal_x, goal_y, linear_tolerance, angular_tolerance):
        # Similar to previous move_to_pose logic but just for a point (no final orientation)
        self.linear_pid.reset(sp=0)
        self.angular_pid.reset(sp=0)

        for i in range(200):
            pos = self.get_position()
            if pos is None:
                continue
            rx, ry, rtheta = pos

            dx = goal_x - rx
            dy = goal_y - ry
            dist_error = math.sqrt(dx*dx + dy*dy)

            if dist_error < linear_tolerance:
                self.power((0,0),0)
                return True

            desired_heading = math.atan2(dy, dx)
            heading_error = self._angle_diff(desired_heading, rtheta)

            self.linear_pid.sp = 0  # We want distance to be zero
            self.angular_pid.sp = 0 # We want heading error to be zero

            linear_correction = self.linear_pid.getCorrection(dist_error)
            angular_correction = self.angular_pid.getCorrection(heading_error)

            linear_speed = self._constrain(linear_correction, -0.5, 0.5)
            angular_speed = self._constrain(angular_correction, -0.5, 0.5)

            left_power = linear_speed - angular_speed
            right_power = linear_speed + angular_speed
            self.power((left_power, right_power), 0.1, off=True)

        return False

    def _orient_to_theta(self, target_theta, angular_tolerance):
        # Turn in place to achieve target_theta
        self.angular_pid.reset(sp=0)
        for i in range(100):
            pos = self.get_position()
            if pos is None:
                continue
            rx, ry, rtheta = pos
            error = self._angle_diff(target_theta, rtheta)
            if abs(error) < angular_tolerance:
                self.power((0,0),0)
                return True

            correction = self.angular_pid.getCorrection(error)
            rotation_speed = self._constrain(correction, -0.3, 0.3)
            self.power((-rotation_speed, rotation_speed), 0.1, off=True)
        
        return False

    @staticmethod
    def _angle_diff(a, b):
        diff = (a - b) % (2*math.pi)
        if diff > math.pi:
            diff -= 2*math.pi
        return diff

    @staticmethod
    def _constrain(value, mn, mx):
        return max(mn, min(mx, value))
