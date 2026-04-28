"""
src/trajectory_generator.py
Waypoint-based trajectory generator for quadruped navigation.
"""

import numpy as np


class WaypointGenerator:
    """
    Iterates through a list of 2-D / 3-D waypoints and returns the
    current target as a [x, y, z, yaw] reference.

    Parameters
    ----------
    waypoints : list of [x, y, z, yaw]
    distance_threshold : float
        Euclidean (XY) radius to consider a waypoint reached [m].
    """

    def __init__(self, waypoints, distance_threshold: float = 0.15):
        self.waypoints     = np.array(waypoints, dtype=float)
        self.current_index = 0
        self.threshold     = distance_threshold
        self.is_finished   = False

    # ------------------------------------------------------------------
    def get_reference(self, current_x: float, current_y: float) -> np.ndarray:
        """
        Returns the current target waypoint [x, y, z, yaw].
        Advances to the next waypoint if the robot is within the threshold.
        """
        if self.is_finished:
            return self.waypoints[-1]

        target   = self.waypoints[self.current_index]
        error_x  = target[0] - current_x
        error_y  = target[1] - current_y
        distance = float(np.sqrt(error_x ** 2 + error_y ** 2))   # BUG FIX

        if distance < self.threshold:
            print(
                f"[Trajectory] Waypoint {self.current_index + 1}/"
                f"{len(self.waypoints)} reached: "
                f"({target[0]:.2f}, {target[1]:.2f})"
            )
            self.current_index += 1

            if self.current_index >= len(self.waypoints):
                print("[Trajectory] Route complete! Holding final position.")
                self.is_finished   = True
                self.current_index = len(self.waypoints) - 1

        return self.waypoints[self.current_index]

    # ------------------------------------------------------------------
    def progress(self):
        """Returns (current_index, total_waypoints, is_finished)."""
        return self.current_index, len(self.waypoints), self.is_finished

    # ------------------------------------------------------------------
    def reset(self):
        """Restart the trajectory from the first waypoint."""
        self.current_index = 0
        self.is_finished   = False
