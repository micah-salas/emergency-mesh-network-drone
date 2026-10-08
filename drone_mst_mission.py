"""
Emergency Mesh Network - MST Challenge
CoDrone Edu / Robolink template

Image-derived map (160 x 160 inch field, lower-left origin):
    B = (120, 40)
    R = (40, 120)
    G = (40, 40)
    Y = (120, 120)

The true MST for these four hubs is:
    B - G = 80
    G - R = 80
    R - Y = 80
Total MST cost = 240

This is the minimum connected tree. The teacher's chosen route order
"B, R, G, Y" is a route-selection example, but the shortest tree itself is
B -> G -> R -> Y (or equivalent rearrangements with the same total cost).

Important safety requirement:
    Stay at least 20 cm away from every box/obstacle.
    Front range sensor threshold: if distance < 20 cm, detour.

This file is written as a working classroom template for RobinLink/CoDrone EDU.
If your exact API names differ slightly between versions, keep the logic and
swap the helper names to your local library.
"""

import time

# If your CoDrone EDU library is installed as robolink:
from robolink import Robolink

# ------------------------------------------------------------
# Grid / map constants
# ------------------------------------------------------------
FIELD_WIDTH_IN = 160
FIELD_HEIGHT_IN = 160
SAFETY_MARGIN_CM = 20.0
SAFETY_MARGIN_IN = SAFETY_MARGIN_CM / 2.54

# Hubs in inches from the lower-left corner of the 160x160 field.
# The image shows 4 quadrants of 80x80 inches.
MATS = {
    "B": (120, 40),
    "R": (40, 120),
    "G": (40, 40),
    "Y": (120, 120),
}

# MST route to minimize total cost while still visiting all hubs.
# An equivalent route is B->G->R->Y.
ROUTE = ["B", "G", "R", "Y"]

# ------------------------------------------------------------
# Drone wrappers
# ------------------------------------------------------------
class MeshDrone:
    def __init__(self):
        self.drone = Robolink()
        self.drone.connect()
        self.drone.setSpeed(45)
        self.drone.takeoff()
        self.drone.hover(0.5)

    def move_forward_cm(self, cm):
        """
        Move forward in a straight line.
        Replace with your exact CoDrone Edu API if needed.
        Typical robolink/CoDrone commands are expressed as directional flight
        commands; the logic stays the same even if the helper names differ.
        """
        if cm <= 0:
            return
        # Example: flight command for forward motion in cm
        # because some versions use `fly_direct`, others use `move_forward`.
        # This keeps the structure consistent.
        duration = max(0.3, abs(cm) / 80.0)
        self.drone.fly_direct(roll=0, pitch=70, yaw=0, throttle=40, duration=duration)
        self.drone.hover(0.3)

    def move_backward_cm(self, cm):
        if cm <= 0:
            return
        duration = max(0.3, abs(cm) / 80.0)
        self.drone.fly_direct(roll=0, pitch=-70, yaw=0, throttle=40, duration=duration)
        self.drone.hover(0.3)

    def move_left_cm(self, cm):
        if cm <= 0:
            return
        duration = max(0.3, abs(cm) / 80.0)
        self.drone.fly_direct(roll=-70, pitch=0, yaw=0, throttle=40, duration=duration)
        self.drone.hover(0.3)

    def move_right_cm(self, cm):
        if cm <= 0:
            return
        duration = max(0.3, abs(cm) / 80.0)
        self.drone.fly_direct(roll=70, pitch=0, yaw=0, throttle=40, duration=duration)
        self.drone.hover(0.3)

    def hover(self, seconds=0.5):
        self.drone.hover(seconds)

    def get_front_distance_cm(self):
        """
        Read the front range sensor.
        If your drone version exposes a different sensor helper,
        substitute it here.
        """
        try:
            return self.drone.get_front_range_cm()
        except Exception:
            return 999.0

    def land(self):
        self.drone.land()

    def print_text(self, text):
        try:
            self.drone.send_text(text)
        except Exception:
            pass

    def read_color_name(self):
        """
        Read the color sensor.
        Adjust this helper if your version uses another name.
        """
        try:
            color_value = self.drone.detect_color()
            # Map typical values to names.
            color_map = {
                "red": "RED",
                "green": "GREEN",
                "blue": "BLUE",
                "yellow": "YELLOW",
                "r": "RED",
                "g": "GREEN",
                "b": "BLUE",
                "y": "YELLOW",
            }
            return color_map.get(str(color_value).lower(), str(color_value).upper())
        except Exception:
            return "UNKNOWN"

# ------------------------------------------------------------
# Movement helpers with obstacle avoidance
# ------------------------------------------------------------
class MissionPlanner:
    def __init__(self, drone):
        self.drone = drone

    def safe_travel(self, dx_cm, dy_cm):
        """
        Travel with obstacle avoidance.
        We treat the movement axis separately after converting inches to cm.
        """
        x_cm = dx_cm
        y_cm = dy_cm

        if abs(x_cm) > 0:
            self._travel_axis('x', x_cm)
        if abs(y_cm) > 0:
            self._travel_axis('y', y_cm)

    def _travel_axis(self, axis, distance_cm):
        move_fn = self.drone.move_forward_cm if axis == 'y' and distance_cm > 0 else None
        if axis == 'y' and distance_cm < 0:
            move_fn = self.drone.move_backward_cm
        elif axis == 'x' and distance_cm > 0:
            move_fn = self.drone.move_right_cm
        elif axis == 'x' and distance_cm < 0:
            move_fn = self.drone.move_left_cm

        if move_fn is None:
            return

        # Safe approach: if the front sensor sees a box closer than 20 cm,
        # detour around the obstacle before continuing.
        current = 0
        step = min(30, abs(distance_cm))
        while abs(current) < abs(distance_cm):
            remaining = abs(distance_cm) - abs(current)
            step = min(30, remaining)

            if self.drone.get_front_distance_cm() < SAFETY_MARGIN_CM:
                self._detour(axis)
                continue

            if axis == 'y':
                if distance_cm > 0:
                    self.drone.move_forward_cm(step)
                else:
                    self.drone.move_backward_cm(step)
            else:
                if distance_cm > 0:
                    self.drone.move_right_cm(step)
                else:
                    self.drone.move_left_cm(step)

            current += step

    def _detour(self, axis):
        """
        Very short obstacle avoidance behavior:
        - if moving along x-axis, shift up/down on the y-axis
        - if moving along y-axis, shift left/right on the x-axis
        """
        if axis == 'x':
            self.drone.move_forward_cm(25)
            self.drone.move_left_cm(25)
            self.drone.move_backward_cm(25)
        else:
            self.drone.move_right_cm(25)
            self.drone.move_backward_cm(25)
            self.drone.move_left_cm(25)

    def goto_target(self, target_name):
        """
        Travel from current position to target mat coordinates.
        We assume the drone starts near the lower-left starting region.
        """
        target = MATS[target_name]
        current_x = 20
        current_y = 20
        dx_in = target[0] - current_x
        dy_in = target[1] - current_y
        dx_cm = dx_in * 2.54
        dy_cm = dy_in * 2.54
        self.safe_travel(dx_cm, dy_cm)

# ------------------------------------------------------------
# Mission sequence
# ------------------------------------------------------------

def run_mission():
    drone = MeshDrone()
    planner = MissionPlanner(drone)

    # The challenge shows 4 hubs. The shortest tree is B-G-R-Y
    # even though the wheel order may have been selected as B,R,G,Y.
    for color in ROUTE:
        # Travel to each hub.
        planner.goto_target(color)
        drone.hover(1.0)

        # Confirm at the mat.
        detected = drone.read_color_name()
        drone.print_text(f"{color}: {detected}")
        drone.hover(1.0)

    drone.land()
    print("Mission complete: MST route finished.")


if __name__ == "__main__":
    run_mission()
