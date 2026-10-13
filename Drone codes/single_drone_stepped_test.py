#!/usr/bin/env python3
"""
Single-drone flight test for PX4 + ROS 2 (real hardware, ONE drone).

Standalone: no formation, no neighbors, no SITL, no GPS-origin setup.
It uses PX4's own local position estimate (which already fuses GPS),
so there is nothing to configure about the field's coordinates.

STEPPED SETPOINT IDEA
    Real motors are slower than Gazebo, so instead of sending the final
    target at once, the commanded position is walked toward it in small
    increments every control tick:

        setpoint += clamp(target - setpoint, -STEP, +STEP)

    Speed of the ramp = STEP / CONTROL_PERIOD_SEC  (0.1 m / 0.1 s = 1 m/s).

FLIGHT SEQUENCE
    WAIT_FOR_DATA -> TAKEOFF -> HOVER -> PATH (repeated) -> HOLD -> LAND
    1. TAKEOFF : only Z ramps up. X and Y are frozen.
    2. HOVER   : settle at height for HOVER_SECONDS.
    3. PATH    : X/Y ramp through the left/right waypoints, pausing
                 WAYPOINT_HOLD_SEC at each one. The whole left/right
                 pattern is repeated PATH_REPEATS times.
    4. HOLD    : hover for FINAL_HOVER_SECONDS after the last repeat.
    5. LAND    : automatic after HOLD (AUTO_LAND = True), or earlier if
                 a waypoint times out. You can ALWAYS land sooner by
                 pressing ENTER in this terminal, or by using the RC /
                 QGroundControl Land button.

DIRECTIONS
    Yaw is held at 0 (nose pointing North). "Left" = West, "right" =
    East, as seen from the drone's back. Change PATH_OFFSETS to alter it.

SAFETY (read before flying)
    - Have the RC transmitter ready to take manual control / switch
      out of Offboard at any time. This script is NOT a failsafe.
    - Test with the props OFF first, then tethered or low, in an open area.
    - Start with small STEP values; raise only if tracking looks good.

USAGE
    python3 single_drone_stepped_test.py

TOPICS
    Check with `ros2 topic list | grep fmu`. If your topics look like
    /drone0/fmu/..., set NAMESPACE = "drone0" below. If they are bare
    (/fmu/...), leave it "". The versioned names below match your
    earlier setup; re-check them if you rebuild against another PX4.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# ---------------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------------

NAMESPACE = ""            # "" for bare /fmu/..., or e.g. "drone0" for /drone0/fmu/...
MAV_SYS_ID = 2            # verify with `param show MAV_SYS_ID` in the PX4 shell

LOCAL_POSITION_TOPIC = "vehicle_local_position_v1"
VEHICLE_STATUS_TOPIC = "vehicle_status_v1"   # real drones use v1 (laptop SITL uses v4)

# --- Stepped setpoint (ramp) ---
STEP_Z = 0.05             # m per tick (0.05 -> 0.5 m/s climb). Raise slowly.
STEP_XY = 0.05            # m per tick (0.05 -> 0.5 m/s sideways)
MAX_SETPOINT_LEAD = 0.5   # m: setpoint may never run further than this ahead
                          # of the drone's REAL position (stops runaway/windup)

# --- Mission ---
TAKEOFF_HEIGHT = 2.0      # m above the spot where the drone starts
ARRIVAL_THRESHOLD = 0.3   # m: how close (real position) counts as "arrived"
HOVER_SECONDS = 5.0       # settle time after reaching height

LATERAL_DISTANCE = 1.5    # m: how far left / right to go (1-2 m as planned)
# (North, East) offsets from the takeoff point, visited in order.
# left = West (-East), right = East (+East).
PATH_OFFSETS = [
    (0.0, -LATERAL_DISTANCE),   # left
    (0.0, 0.0),                 # back to centre
    (0.0, +LATERAL_DISTANCE),   # right
    (0.0, 0.0),                 # back to centre
]
PATH_REPEATS = 3          # how many times the whole left/right pattern is flown
WAYPOINT_HOLD_SEC = 2.0   # pause at each waypoint
WAYPOINT_TIMEOUT_SEC = 20.0  # if a waypoint isn't reached in this time -> land (or hold, see AUTO_LAND)

FINAL_HOVER_SECONDS = 5.0 # hover this long after the last repeat, then land
AUTO_LAND = True          # False = never land by itself: hover until you press ENTER / use RC

CONTROL_PERIOD_SEC = 0.1  # 10 Hz (PX4 needs >= 2 Hz offboard stream)

# ---------------------------------------------------------------------------


def step_toward(current: float, target: float, step: float) -> float:
    """Move `current` toward `target` by at most `step` (never overshoots)."""
    delta = target - current
    if delta > step:
        return current + step
    if delta < -step:
        return current - step
    return target


class SingleDroneSteppedTest(Node):
    def __init__(self):
        super().__init__("single_drone_stepped_test")

        prefix = f"/{NAMESPACE}" if NAMESPACE else ""

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, f"{prefix}/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f"{prefix}/fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, f"{prefix}/fmu/in/vehicle_command", qos_profile)

        self.create_subscription(
            VehicleLocalPosition, f"{prefix}/fmu/out/{LOCAL_POSITION_TOPIC}",
            self.local_position_callback, qos_profile)
        self.create_subscription(
            VehicleStatus, f"{prefix}/fmu/out/{VEHICLE_STATUS_TOPIC}",
            self.vehicle_status_callback, qos_profile)

        # --- State ---
        self.pos = None             # real position, PX4 local NED [N, E, D]
        self.vehicle_status = VehicleStatus()
        self.sp = None              # ramped setpoint, PX4 local NED [N, E, D]
        self.start = None           # [N, E, D] where the drone started
        self.state = "WAIT_FOR_DATA"
        self.offboard_setpoint_counter = 0
        self.offboard_engaged = False
        self.state_time = 0.0       # seconds spent in the current state / waypoint
        self.wp_index = 0
        self.wp_reached = False
        self.cycle = 0              # how many full left/right patterns are done
        self.warned_not_offboard = False
        self.land_requested = False

        # Press ENTER in this terminal to land. Runs in a background thread
        # so it never blocks the control loop. (If there is no terminal
        # attached, the thread just exits and you use RC / QGC to land.)
        threading.Thread(target=self.wait_for_enter, daemon=True).start()

        self.timer = self.create_timer(CONTROL_PERIOD_SEC, self.control_loop)
        self.get_logger().info("Single-drone stepped test node started. Waiting for valid position...")

    # -----------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------

    def local_position_callback(self, msg: VehicleLocalPosition):
        # Only trust the estimate once PX4 says it is valid.
        if msg.xy_valid and msg.z_valid:
            self.pos = [msg.x, msg.y, msg.z]

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    # -----------------------------------------------------------------
    # State machine
    # -----------------------------------------------------------------

    def set_state(self, new_state: str):
        self.get_logger().info(f"State: {self.state} -> {new_state}")
        self.state = new_state
        self.state_time = 0.0

    def wait_for_enter(self):
        try:
            input()
            self.land_requested = True
        except (EOFError, OSError):
            pass

    def target_for_state(self):
        """Local-NED [N, E, D] the ramp should walk toward in this state."""
        sx, sy, sz = self.start
        if self.state == "TAKEOFF":
            # Straight up. X/Y stay at the starting point.
            return [sx, sy, sz - TAKEOFF_HEIGHT]
        if self.state == "HOVER":
            return [sx, sy, sz - TAKEOFF_HEIGHT]
        if self.state == "PATH":
            dn, de = PATH_OFFSETS[self.wp_index]
            return [sx + dn, sy + de, sz - TAKEOFF_HEIGHT]
        return list(self.sp)  # HOLD: stay exactly where the setpoint is

    def advance_setpoint(self, target):
        """
        One ramp step per axis, then clamp so the setpoint stays within
        MAX_SETPOINT_LEAD of the real position. During TAKEOFF only Z moves.
        """
        for axis in range(3):
            if self.state == "TAKEOFF" and axis != 2:
                continue
            step = STEP_Z if axis == 2 else STEP_XY
            new_sp = step_toward(self.sp[axis], target[axis], step)
            lo = self.pos[axis] - MAX_SETPOINT_LEAD
            hi = self.pos[axis] + MAX_SETPOINT_LEAD
            self.sp[axis] = max(lo, min(hi, new_sp))

    def update_state(self):
        """Transitions, based on the drone's REAL position (not the setpoint)."""
        self.state_time += CONTROL_PERIOD_SEC
        sx, sy, sz = self.start

        if self.state == "TAKEOFF":
            if abs((sz - TAKEOFF_HEIGHT) - self.pos[2]) < ARRIVAL_THRESHOLD:
                self.set_state("HOVER")

        elif self.state == "HOVER":
            if self.state_time >= HOVER_SECONDS:
                self.wp_index = 0
                self.wp_reached = False
                self.cycle = 0
                self.set_state("PATH")

        elif self.state == "PATH":
            dn, de = PATH_OFFSETS[self.wp_index]
            dist = math.hypot(sx + dn - self.pos[0], sy + de - self.pos[1])

            if not self.wp_reached:
                if dist < ARRIVAL_THRESHOLD:
                    self.wp_reached = True
                    self.state_time = 0.0
                    self.get_logger().info(f"Waypoint {self.wp_index} reached, holding")
                elif self.state_time > WAYPOINT_TIMEOUT_SEC:
                    self.sp = list(self.pos)   # stay where the drone really is
                    if AUTO_LAND:
                        self.get_logger().warn(f"Waypoint {self.wp_index} timed out -> LAND")
                        self.start_landing()
                    else:
                        self.get_logger().warn(
                            f"Waypoint {self.wp_index} timed out -> HOLD (hovering in place)")
                        self.set_state("HOLD")
                        self.announce_hold()
            elif self.state_time >= WAYPOINT_HOLD_SEC:
                self.wp_index += 1
                self.wp_reached = False
                self.state_time = 0.0
                if self.wp_index >= len(PATH_OFFSETS):
                    self.cycle += 1
                    if self.cycle < PATH_REPEATS:
                        self.get_logger().info(
                            f"Pattern {self.cycle}/{PATH_REPEATS} done, repeating")
                        self.wp_index = 0
                    else:
                        self.get_logger().info(
                            f"All {PATH_REPEATS} patterns done -> HOLD")
                        self.set_state("HOLD")
                        self.announce_hold()

        elif self.state == "HOLD":
            if AUTO_LAND and self.state_time >= FINAL_HOVER_SECONDS:
                self.get_logger().info("Final hover done -> LAND")
                self.start_landing()

    def announce_hold(self):
        if AUTO_LAND:
            self.get_logger().info(
                f"HOLDING for {FINAL_HOVER_SECONDS:.0f} s, then landing. "
                "Press ENTER here to land now, or use RC / QGC.")
        else:
            self.get_logger().info(
                "HOLDING position. Press ENTER here to land, or use RC / QGC.")

    def start_landing(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.set_state("LAND")

    # -----------------------------------------------------------------
    # Main loop
    # -----------------------------------------------------------------

    def control_loop(self):
        self.publish_offboard_control_mode()

        # Need a valid position before we can command anything. The
        # heartbeat above still runs, but no setpoints / arm / offboard yet.
        if self.state == "WAIT_FOR_DATA":
            if self.pos is None:
                return
            self.start = list(self.pos)
            self.sp = list(self.pos)   # ramp starts exactly where we are: no jump
            self.get_logger().info(
                f"Start position (local NED) = {[round(v, 2) for v in self.start]}")
            self.set_state("TAKEOFF")

        # Stream setpoints for a moment first, then engage offboard + arm.
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.offboard_engaged = True

        if self.land_requested and self.state != "LAND":
            self.get_logger().info("ENTER pressed -> sending land command")
            self.start_landing()

        if self.state != "LAND":
            self.update_state()

        if self.state != "LAND":
            self.advance_setpoint(self.target_for_state())
            self.publish_trajectory_setpoint(self.sp)

        self.check_still_in_offboard()

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def check_still_in_offboard(self):
        """Warn once if PX4 left Offboard mode (e.g. you took over with RC)."""
        if not self.offboard_engaged or self.offboard_setpoint_counter < 30:
            return
        if (self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and not self.warned_not_offboard and self.state != "LAND"):
            self.warned_not_offboard = True
            self.get_logger().warn(
                "PX4 is no longer in Offboard mode (RC takeover or failsafe?)")

    # -----------------------------------------------------------------
    # Publishing helpers
    # -----------------------------------------------------------------

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True        # stepped POSITION setpoints
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, position_local):
        msg = TrajectorySetpoint()
        msg.position = [float(position_local[0]), float(position_local[1]), float(position_local[2])]
        nan3 = [float("nan")] * 3
        msg.velocity = list(nan3)      # no velocity feed-forward
        msg.acceleration = list(nan3)
        msg.jerk = list(nan3)
        msg.yaw = 0.0                  # nose North, so left = West, right = East
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = MAV_SYS_ID
        msg.target_component = 1
        msg.source_system = MAV_SYS_ID
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")


def main():
    rclpy.init()
    node = SingleDroneSteppedTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
