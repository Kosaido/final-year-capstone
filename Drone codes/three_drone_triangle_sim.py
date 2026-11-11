#!/usr/bin/env python3
"""
Three-drone TRIANGLE formation + collision avoidance: SIMULATION (SITL) VERSION
(PX4 SITL + Gazebo + ROS 2 on one PC, drones talk over localhost UDP)

THIS FILE = the simulation version: DEPLOYMENT SITL, world-frame positions from the spawn offsets, no
height stagger. Its sister file three_drone_triangle_real.py has identical control code; only the
CONFIGURATION block and this text differ.

SITL SETUP (three PX4 instances, one Micro XRCE-DDS agent)
    - SPAWN_WORLD_OFFSET below MUST equal each instance's PX4_GZ_MODEL_POSE (East, North):
        -i 0  "0,6"    (leader, North)      -i 1  "-3,0"    (follower, West)      -i 2  "3,0"    (follower, East)
      Start drone 0 first (it opens Gazebo); instances 1 and 2 need PX4_GZ_STANDALONE=1.
    - Instance -i 0 -> topics /fmu/..., -i 1 -> /px4_1/fmu/..., -i 2 -> /px4_2/fmu/...
    - Check VEHICLE_STATUS_TOPIC / LOCAL_POSITION_TOPIC against `ros2 topic list`.

THE FORMATION
    Drone 0 = LEADER (front). Drones 1 and 2 = FOLLOWERS, behind it to the left and right:
                    0
                  /   \\            side length s (default 4 m), triangle points North
                1 ----- 2
    Followers keep a fixed offset to the leader (displacement-based control, survey Eq. 12/17).
    ALL three drones avoid each other (potential field of Dimarogonas & Kyriakopoulos 2006),
    each drone adding up the push from both neighbours.

THE TEST SCHEDULE (PLAN below; drone 0 owns it and broadcasts the current step)
    TRIANGLE    static triangle, 4 m sides
    SQUEEZE     asks for 2 m sides (inside the avoidance zone: formation and avoidance fight)
    TRIANGLE    back to 4 m
    SWAP        the two followers exchange corners, so their paths cross
    MOVE_N      leader walks North slowly, followers keep the triangle (moving formation)
    MOVE_S      leader walks back South, still with the swapped corners
    HOLD        settle, then everyone lands

HEARTBEAT FIX (new)
    Earlier real flights (logs 202-204) lost the offboard heartbeat for ~1 s and PX4 raised
    "offboard control signal lost". Here the heartbeat AND the latest setpoint are published by a
    separate 20 Hz timer in its own callback group (MultiThreadedExecutor), so a slow control tick
    cannot stop them. If the control loop stalls for more than CMD_FRESH_SEC, that timer sends a
    ZERO-velocity hold instead of repeating an old command. Late ticks are counted and logged
    ("heartbeat late"), which helps find the real cause if dropouts continue.

USAGE (one terminal per drone, on that drone's Pi)
    python3 three_drone_triangle_sim.py --id 0 --plan formation|avoidance|full     # leader, owns the schedule
    python3 three_drone_triangle_sim.py --id 1      # follower, left
    python3 three_drone_triangle_sim.py --id 2      # follower, right
    Place the drones roughly in the triangle (leader North of both, drone 1 West of drone 2),
    3 m or more apart. Press ENTER in each terminal to arm + take off; ENTER again to land.

SAFETY
    - Props OFF first. RC transmitter in hand for EVERY drone. This script is not a failsafe.
    - First flights: ALT_STAGGER gives each drone a different height. Avoidance only looks at
      horizontal distance, so the stagger is a pure safety margin. Set it to 0 only when confident.
    - Any drone landing, or any link lost for LINK_LOST_LAND_SEC, lands all drones. While a link is
      stale the drones hold position. Ctrl+C sends a land command.
    - Yaw is held at each drone's starting heading (no rotation at takeoff).

LOG
    triangle_log_drone<ID>_<time>.csv in the current folder (10 Hz).
"""

import argparse
import csv
import json
import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleGlobalPosition,
    VehicleStatus,
)

# ---------------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------------

DEPLOYMENT = "SITL"   # fixed for this file (see three_drone_triangle_real.py for hardware)

# --- Topics (check with `ros2 topic list | grep fmu/out`) ---
LOCAL_POSITION_TOPIC = "vehicle_local_position_v1"
GLOBAL_POSITION_TOPIC = "vehicle_global_position"      # REAL only; may carry a version suffix
VEHICLE_STATUS_TOPIC = "vehicle_status_v1" if DEPLOYMENT == "REAL" else "vehicle_status_v4"
REAL_NAMESPACE = ""   # every Pi sees only its own drone: bare /fmu/...

# --- REAL: each vehicle's actual MAV_SYS_ID (`param show MAV_SYS_ID`). Wrong value = commands ignored.
REAL_MAV_SYS_ID = {0: 1, 1: 2, 2: 3}

DRONE_IDS = (0, 1, 2)
LEADER_ID = 0
MASTER_ID = 0          # owns the test schedule

# --- UDP link between the three drones. REAL: each Pi's IP (`hostname -I`), same Wi-Fi/router. ---
PEER_IP = ({0: "CHANGE_ME", 1: "CHANGE_ME", 2: "CHANGE_ME"} if DEPLOYMENT == "REAL"
           else {0: "127.0.0.1", 1: "127.0.0.1", 2: "127.0.0.1"})
BASE_PORT = 14800            # drone N listens on BASE_PORT + N
NEIGHBOR_TIMEOUT_SEC = 0.5   # older than this = stale
LINK_LOST_LAND_SEC = 5.0     # any neighbour stale this long while flying -> land everyone
MAX_EXTRAPOLATE_SEC = 0.5

# --- SITL only: spawn positions in the shared world (East, North), metres.
#     Must equal PX4_GZ_MODEL_POSE of each instance. Leader North of both, drone 1 West of drone 2.
SPAWN_WORLD_OFFSET = {0: (0.0, 6.0), 1: (-3.0, 0.0), 2: (3.0, 0.0)}

# --- Test schedules: (label, side length m, followers swap corners, leader vE, leader vN, hold s)
# Choose one with --plan on DRONE 0's command (drone 0 owns the schedule; the others just follow it).
PLANS = {
    # formation only: static triangle, then the leader walks North and back with the followers keeping up
    "formation": [
        ("TRIANGLE", 4.0, False, 0.0, 0.0, 30.0),
        ("MOVE_N", 4.0, False, 0.0, 0.3, 12.0),
        ("MOVE_S", 4.0, False, 0.0, -0.3, 12.0),
        ("HOLD", 4.0, False, 0.0, 0.0, 10.0),
    ],
    # avoidance inside the formation: squeeze to 2 m sides, then the followers swap corners (paths cross)
    "avoidance": [
        ("TRIANGLE", 4.0, False, 0.0, 0.0, 20.0),
        ("SQUEEZE", 2.0, False, 0.0, 0.0, 20.0),
        ("TRIANGLE", 4.0, False, 0.0, 0.0, 15.0),
        ("SWAP", 4.0, True, 0.0, 0.0, 35.0),
        ("HOLD", 4.0, True, 0.0, 0.0, 10.0),
    ],
    # everything in one flight
    "full": [
        ("TRIANGLE", 4.0, False, 0.0, 0.0, 30.0),
        ("SQUEEZE", 2.0, False, 0.0, 0.0, 20.0),
        ("TRIANGLE", 4.0, False, 0.0, 0.0, 15.0),
        ("SWAP", 4.0, True, 0.0, 0.0, 35.0),
        ("MOVE_N", 4.0, True, 0.0, 0.3, 12.0),
        ("MOVE_S", 4.0, True, 0.0, -0.3, 12.0),
        ("HOLD", 4.0, True, 0.0, 0.0, 10.0),
    ],
}
FORMATION_PLAN = PLANS["full"]
AUTO_LAND = True

# --- Mission ---
TAKEOFF_HEIGHT = 2.0
ALT_STAGGER = {0: 0.0, 1: 0.0, 2: 0.0}   # extra metres per drone (safety margin); not needed in simulation
ARRIVAL_THRESHOLD = 0.3
ALLOW_ANY_START_LAYOUT = False   # False: refuse takeoff unless leader is North of both and drone 1 West of drone 2
MIN_START_SEPARATION = 3.0
MAX_START_SEPARATION = 30.0

# --- Formation gains (survey Eq. 17) ---
KP = 0.4
KV = 0.5
FF_GAIN = 1.0                  # followers add the leader's velocity so they keep up while it moves
FORMATION_MAX_SPEED = 0.6      # cap on the formation pull BEFORE avoidance is added (< REPULSION_SPEED_AT_CRITICAL)
MAX_SPEED_XY = 0.8
MAX_SPEED_Z = 0.6
ALT_KP = 0.5
ANCHOR_KP = 0.5
ANCHOR_MAX_SPEED = 0.5         # also the fastest the leader may walk in the plan
MAX_ANCHOR_LEAD = 1.0          # the leader's moving target never runs further than this ahead of it

# --- Collision avoidance potential (Dimarogonas 2006) ---
D_SENSE = 3.0                  # avoidance is exactly zero beyond this distance; keep the side length above it
REPULSION_SPEED_AT_CRITICAL = 1.0
AVOID_SWIRL_DEG = 30.0         # 30 = also sidestep to the right (needed for the SWAP step: head-on drones stall without it).
                               # 0 = exactly the paper.
assert FORMATION_MAX_SPEED < REPULSION_SPEED_AT_CRITICAL, "avoidance must out-push the formation at the critical radius"
assert all(math.hypot(st[3], st[4]) <= ANCHOR_MAX_SPEED for pl in PLANS.values() for st in pl), "leader plan speed above ANCHOR_MAX_SPEED"
_C = D_SENSE ** 2 / 3.0                                   # paper: d^2 = 3c
REP_H = REPULSION_SPEED_AT_CRITICAL / (8.0 * _C ** 1.5)   # paper: a = 4*h*c^3

# --- Heartbeat / setpoint publisher (separate fast timer) ---
FAST_PERIOD_SEC = 0.05         # 20 Hz heartbeat + setpoint
CMD_FRESH_SEC = 0.35           # control command older than this -> send a zero-velocity hold instead
LATE_FAST_SEC = 0.15           # a fast tick later than this is counted as "heartbeat late"

TAKEOFF_TIMEOUT_SEC = 25.0
WAIT_PEER_TIMEOUT_SEC = 90.0
CONTROL_PERIOD_SEC = 0.1

AIRBORNE_PHASES = ("TAKEOFF", "WAIT_PEER", "FORMATION")
R_EARTH = 6371000.0

# ---------------------------------------------------------------------------
# PURE MATH
# ---------------------------------------------------------------------------


def ll_to_en_diff(lat_a, lon_a, lat_b, lon_b):
    """(east, north) of A relative to B in metres (flat earth, fine at tens of metres)."""
    north = math.radians(lat_a - lat_b) * R_EARTH
    east = math.radians(lon_a - lon_b) * R_EARTH * math.cos(math.radians((lat_a + lat_b) / 2.0))
    return east, north


def clamp(x, lim):
    return max(-lim, min(lim, x))


def clamp_vec(v, vmax):
    n = math.hypot(v[0], v[1])
    if n > vmax and n > 0.0:
        return (v[0] * vmax / n, v[1] * vmax / n)
    return (v[0], v[1])


def potential_rho(beta, d, h):
    """rho = dV/dbeta; V = a/beta (beta<c), h(beta-d^2)^2 (c<=beta<d^2), 0 beyond; d^2=3c, a=4hc^3."""
    c = d * d / 3.0
    if beta >= d * d:
        return 0.0
    if beta >= c:
        return 2.0 * h * (beta - d * d)
    return -(4.0 * h * c ** 3) / (beta * beta)


def repulsion_velocity(rel, d, h, fallback_sign, swirl_deg=0.0):
    """-dV/dq_i = -2 rho (q_i - q_j): pushes drone i straight away from drone j. rel = q_i - q_j."""
    e, n = rel
    if math.hypot(e, n) < 0.05:
        e, n = fallback_sign * 0.05, 0.0
    beta = max(e * e + n * n, 1e-6)
    k = -2.0 * potential_rho(beta, d, h)
    ue, un = k * e, k * n
    if swirl_deg:
        a = math.radians(swirl_deg)
        ue, un = ue * math.cos(a) - un * math.sin(a), ue * math.sin(a) + un * math.cos(a)
    return (ue, un)


def formation_velocity(rel, des, v_own, v_nb, kp, kv):
    """u_i = -kp*((q_i - q_j) - (q_i* - q_j*)) - kv*(v_i - v_j)"""
    return (-kp * (rel[0] - des[0]) - kv * (v_own[0] - v_nb[0]),
            -kp * (rel[1] - des[1]) - kv * (v_own[1] - v_nb[1]))


def slot_offset(drone_id, side, swap):
    """Desired q_i* - q_leader* (East, North). Drone 1 is the LEFT corner, drone 2 the RIGHT, unless swapped."""
    if drone_id == LEADER_ID:
        return (0.0, 0.0)
    left = (drone_id == 1) != swap
    return ((-0.5 if left else 0.5) * side, -side * math.sqrt(3.0) / 2.0)


# ---------------------------------------------------------------------------
# UDP LINK
# ---------------------------------------------------------------------------

_REQUIRED_KEYS = ("id", "phase", "e", "n", "ve", "vn", "lat", "lon", "label", "plan_idx", "side", "swap",
                  "lvE", "lvN")


class PeerLink:
    def __init__(self, my_id, listen_port, peer_addrs):
        self.my_id = my_id
        self.peer_addrs = peer_addrs
        self.latest = {}
        self.lock = threading.Lock()
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", listen_port))
        self.sock.settimeout(0.5)
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def send(self, packet):
        data = json.dumps(packet, allow_nan=False).encode()
        for addr in self.peer_addrs:
            try:
                self.sock.sendto(data, addr)
            except OSError:
                pass

    def _rx_loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                pkt = json.loads(data.decode())
                if not isinstance(pkt, dict) or any(k not in pkt for k in _REQUIRED_KEYS):
                    continue
                if pkt["id"] == self.my_id or pkt["id"] not in DRONE_IDS:
                    continue
                for k in ("e", "n", "ve", "vn", "lat", "lon", "side", "lvE", "lvN"):
                    if not isinstance(pkt[k], (int, float)) or not math.isfinite(pkt[k]):
                        raise ValueError(k)
            except (ValueError, UnicodeDecodeError):
                continue
            with self.lock:
                self.latest[pkt["id"]] = (time.monotonic(), pkt)

    def get(self, peer_id):
        """(age_seconds, packet) or None if nothing received yet."""
        with self.lock:
            item = self.latest.get(peer_id)
        if item is None:
            return None
        return (time.monotonic() - item[0], item[1])

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


def get_namespace(i):
    if DEPLOYMENT == "REAL":
        return REAL_NAMESPACE
    return "" if i == 0 else f"/px4_{i}"


def _f(x):
    return float(x) if x is not None and math.isfinite(x) else 0.0


# ---------------------------------------------------------------------------
# NODE
# ---------------------------------------------------------------------------


class TriangleNode(Node):
    def __init__(self, drone_id):
        super().__init__(f"three_drone_triangle_{drone_id}")
        self.drone_id = drone_id
        self.peers = [p for p in DRONE_IDS if p != drone_id]
        self.is_master = drone_id == MASTER_ID
        self.is_leader = drone_id == LEADER_ID
        self.mav_sys_id = REAL_MAV_SYS_ID[drone_id] if DEPLOYMENT == "REAL" else drone_id + 1
        if DEPLOYMENT == "REAL" and "CHANGE_ME" in PEER_IP.values():
            raise RuntimeError("Set PEER_IP for all three drones (hostname -I on each Pi) before running.")

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.main_group = MutuallyExclusiveCallbackGroup()   # control loop + subscriptions
        self.fast_group = MutuallyExclusiveCallbackGroup()   # heartbeat + setpoint publisher only
        ns = get_namespace(drone_id)
        self.offboard_pub = self.create_publisher(OffboardControlMode, f"{ns}/fmu/in/offboard_control_mode", qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, f"{ns}/fmu/in/trajectory_setpoint", qos)
        self.command_pub = self.create_publisher(VehicleCommand, f"{ns}/fmu/in/vehicle_command", qos)
        self.create_subscription(VehicleLocalPosition, f"{ns}/fmu/out/{LOCAL_POSITION_TOPIC}",
                                 self.local_cb, qos, callback_group=self.main_group)
        self.create_subscription(VehicleStatus, f"{ns}/fmu/out/{VEHICLE_STATUS_TOPIC}",
                                 self.status_cb, qos, callback_group=self.main_group)
        if DEPLOYMENT == "REAL":
            self.create_subscription(VehicleGlobalPosition, f"{ns}/fmu/out/{GLOBAL_POSITION_TOPIC}",
                                     self.global_cb, qos, callback_group=self.main_group)
        self.link = PeerLink(drone_id, BASE_PORT + drone_id, [(PEER_IP[p], BASE_PORT + p) for p in self.peers])

        # own state
        self.local_valid, self.last_local_time = False, -1e9
        self.own_local_en = (0.0, 0.0)
        self.own_world, self.own_latlon = None, None
        self.own_ve = self.own_vn = 0.0
        self.own_z = 0.0
        self.heading = None
        self.vehicle_status = VehicleStatus()

        # mission state
        self.phase, self.phase_t0 = "WAIT_DATA", time.monotonic()
        self.go = self.land_requested = False
        self.anchor = (0.0, 0.0)
        self.start_z = self.target_z = 0.0
        self.hold_yaw = 0.0
        self.takeoff_ticks = 0
        self.last_fresh = time.monotonic()
        self.plan_idx, self.plan_t0, self.plan_done = 0, time.monotonic(), False
        self.step = self._step_dict(0)
        self.land_t0 = self.last_resend = 0.0
        self._last_log = {}
        self.dbg = {}
        self.tick = 0
        self._last_control = time.monotonic()

        # fast publisher state
        self._cmd = ((0.0, 0.0, 0.0), -1e9)      # latest control command (NED N,E,D) and when it was made
        self._fast_last = time.monotonic()
        self.late_count = 0
        self.max_late = 0.0
        self.fast_ticks = 0

        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_file = open(f"triangle_log_drone{drone_id}_{stamp}.csv", "w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(["t_s", "phase", "label", "side_des", "swap", "dist_a", "dist_b", "des_e", "des_n",
                           "rel_lead_e", "rel_lead_n", "u_rep", "cmd_e", "cmd_n", "cmd_d", "own_ve", "own_vn",
                           "late_count", "max_nb_age"])
        self.t_start = time.monotonic()

        self.fast_timer = self.create_timer(FAST_PERIOD_SEC, self.fast_loop, callback_group=self.fast_group)
        self.timer = self.create_timer(CONTROL_PERIOD_SEC, self.control_loop, callback_group=self.main_group)
        role = "LEADER" if self.is_leader else ("FOLLOWER left" if drone_id == 1 else "FOLLOWER right")
        self.get_logger().info(
            f"Drone {drone_id} ({role}) started: {DEPLOYMENT}, mav_sys_id={self.mav_sys_id}, "
            f"D_SENSE={D_SENSE} m (critical radius {D_SENSE / math.sqrt(3):.2f} m), heartbeat {1 / FAST_PERIOD_SEC:.0f} Hz")
        if DEPLOYMENT == "REAL" and len(set(ALT_STAGGER.values())) == 1:
            self.get_logger().warn("ALT_STAGGER is equal for all drones: no vertical safety margin.")

    # ------------------------------------------------------------- utils

    @staticmethod
    def _step_dict(idx):
        label, side, swap, lve, lvn, hold = FORMATION_PLAN[idx]
        return dict(label=label, side=side, swap=bool(swap), lvE=lve, lvN=lvn, idx=idx)

    def log_every(self, key, period, level, msg):
        now = time.monotonic()
        if now - self._last_log.get(key, -1e9) >= period:
            self._last_log[key] = now
            getattr(self.get_logger(), level)(msg)

    def set_phase(self, p):
        self.get_logger().info(f"Phase: {self.phase} -> {p}")
        self.phase, self.phase_t0 = p, time.monotonic()

    # --------------------------------------------------------- callbacks

    def local_cb(self, msg):
        self.last_local_time = time.monotonic()
        self.local_valid = bool(msg.xy_valid and msg.z_valid)
        if not self.local_valid:
            return
        self.own_local_en = (msg.y, msg.x)            # PX4 local NED (x=N, y=E) -> (East, North)
        self.own_ve, self.own_vn, self.own_z = msg.vy, msg.vx, msg.z
        if math.isfinite(msg.heading):
            self.heading = msg.heading
        if DEPLOYMENT == "SITL":
            sx, sy = SPAWN_WORLD_OFFSET[self.drone_id]
            self.own_world = (sx + msg.y, sy + msg.x)

    def global_cb(self, msg):
        self.own_latlon = (msg.lat, msg.lon) if msg.lat_lon_valid else None

    def status_cb(self, msg):
        self.vehicle_status = msg

    def own_ready(self, now):
        if not self.local_valid or self.heading is None or now - self.last_local_time > 1.0:
            return False
        return (self.own_latlon is not None) if DEPLOYMENT == "REAL" else (self.own_world is not None)

    # ------------------------------------------------------ neighbours

    def neighbors(self):
        """{peer_id: (age, packet)} for fresh neighbours only."""
        out = {}
        for p in self.peers:
            item = self.link.get(p)
            if item is not None and item[0] <= NEIGHBOR_TIMEOUT_SEC:
                out[p] = item
        return out

    def relative_to(self, pkt, age):
        """q_me - q_other (East, North), the other drone predicted forward by v*age."""
        if DEPLOYMENT == "REAL":
            e, n = ll_to_en_diff(self.own_latlon[0], self.own_latlon[1], pkt["lat"], pkt["lon"])
        else:
            e, n = self.own_world[0] - pkt["e"], self.own_world[1] - pkt["n"]
        age = min(age, MAX_EXTRAPOLATE_SEC)
        return (e - pkt["ve"] * age, n - pkt["vn"] * age)

    # ----------------------------------------------------- ENTER prompts

    def prompt_takeoff(self):
        def worker():
            try:
                input("\n>>> All three drones in the triangle and clear? Press ENTER to ARM + TAKE OFF: ")
                self.go = True
            except (EOFError, OSError):
                self.get_logger().error("No terminal attached: cannot ask for ENTER, so it will not take off.")
        threading.Thread(target=worker, daemon=True).start()

    def prompt_land(self):
        def worker():
            try:
                input("\n>>> Press ENTER at any time to LAND (lands all three): ")
                self.land_requested = True
            except (EOFError, OSError):
                pass
        threading.Thread(target=worker, daemon=True).start()

    # ---------------------------------------------------- phase machine

    def start_landing(self, reason):
        self.get_logger().warn(f"LANDING: {reason}")
        self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.land_t0 = self.last_resend = time.monotonic()
        self.set_phase("LAND")

    def update_phase(self, now, nbs):
        ph = self.phase
        if ph == "WAIT_DATA":
            if self.own_ready(now):
                self.hold_yaw = self.heading
                self.set_phase("WAIT_GO")
                self.prompt_takeoff()
            else:
                self.log_every("wd", 3.0, "info", "Waiting for a valid position/heading from PX4 (GPS lock)...")
            return
        if ph == "WAIT_GO":
            self.hold_yaw = self.heading if self.heading is not None else self.hold_yaw
            if self.go:
                self.go = False
                self.try_takeoff(now, nbs)
            return
        if ph == "LAND":
            if now - self.land_t0 < 10.0 and now - self.last_resend >= 1.0:
                self.last_resend = now
                self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            return

        # ---- airborne: common safety rules ----
        if len(nbs) == len(self.peers):
            self.last_fresh = now
            if any(pkt["phase"] == "LAND" for _, pkt in nbs.values()):
                self.start_landing("another drone is landing")
                return
        elif now - self.last_fresh > LINK_LOST_LAND_SEC:
            missing = [p for p in self.peers if p not in nbs]
            self.start_landing(f"no data from drone(s) {missing} for {LINK_LOST_LAND_SEC:.0f} s")
            return
        else:
            self.log_every("stale", 1.0, "warn", "A neighbour's data is stale: holding position")
        if self.land_requested:
            self.start_landing("ENTER pressed")
            return
        if self.takeoff_ticks >= 30 and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.log_every("ob", 5.0, "warn", "PX4 is not in Offboard mode (RC takeover, failsafe, or status "
                           "topic not arriving?)")

        if ph == "TAKEOFF":
            self.takeoff_ticks += 1
            if self.takeoff_ticks == 10:
                self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info("Offboard + arm commands sent")
            if abs(self.own_z - self.target_z) < ARRIVAL_THRESHOLD and self.takeoff_ticks > 20:
                self.set_phase("WAIT_PEER")
            elif now - self.phase_t0 > TAKEOFF_TIMEOUT_SEC:
                self.start_landing("takeoff timed out (did it arm? check QGC messages)")
        elif ph == "WAIT_PEER":
            if len(nbs) == len(self.peers) and all(pkt["phase"] in ("WAIT_PEER", "FORMATION") for _, pkt in nbs.values()):
                self.plan_idx, self.plan_t0, self.plan_done = 0, now, False
                self.step = self._step_dict(0)
                self.set_phase("FORMATION")
            elif now - self.phase_t0 > WAIT_PEER_TIMEOUT_SEC:
                self.start_landing("the other drones never became ready")
        elif ph == "FORMATION":
            self.update_plan(now, nbs)

    def update_plan(self, now, nbs):
        if self.is_master:
            hold = FORMATION_PLAN[self.plan_idx][5]
            if not self.plan_done and now - self.plan_t0 >= hold:
                if self.plan_idx + 1 < len(FORMATION_PLAN):
                    self.plan_idx += 1
                    self.plan_t0 = now
                    self.step = self._step_dict(self.plan_idx)
                    self.get_logger().info(f"Plan step -> {self.step['label']} (side {self.step['side']} m, "
                                           f"swap={self.step['swap']}, leader v=({self.step['lvE']}, {self.step['lvN']}))")
                else:
                    self.plan_done = True
                    self.step = dict(self.step, lvE=0.0, lvN=0.0)
                    if AUTO_LAND:
                        self.start_landing("test plan finished")
                    else:
                        self.get_logger().info("Plan finished: holding. ENTER to land.")
        elif MASTER_ID in nbs:
            p = nbs[MASTER_ID][1]
            self.step = dict(label=p["label"], side=p["side"], swap=bool(p["swap"]), lvE=p["lvE"], lvN=p["lvN"],
                             idx=p["plan_idx"])

    def layout_ok(self, nbs):
        """Refuse takeoff if the drones are not where the triangle expects them."""
        rels = {p: self.relative_to(pkt, age) for p, (age, pkt) in nbs.items()}
        for p, r in rels.items():
            d = math.hypot(*r)
            if not (MIN_START_SEPARATION <= d <= MAX_START_SEPARATION):
                self.get_logger().error(f"Refusing takeoff: drone {p} is {d:.1f} m away, outside "
                                        f"[{MIN_START_SEPARATION}, {MAX_START_SEPARATION}] m.")
                return False
        if ALLOW_ANY_START_LAYOUT:
            return True
        if self.is_leader:
            ok = all(rels[p][1] > 0.0 for p in self.peers)        # I must be North of both followers
            why = "the leader (drone 0) must start NORTH of drones 1 and 2"
        else:
            ok = rels[LEADER_ID][1] < 0.0                          # I must be South of the leader
            other = 2 if self.drone_id == 1 else 1
            ok = ok and (rels[other][0] < 0.0 if self.drone_id == 1 else rels[other][0] > 0.0)
            why = "followers must start SOUTH of the leader, drone 1 WEST of drone 2"
        if not ok:
            self.get_logger().error(f"Refusing takeoff: {why}.")
        return ok

    def try_takeoff(self, now, nbs):
        if not self.own_ready(now):
            self.get_logger().error("Refusing takeoff: own position/heading not valid.")
        elif len(nbs) < len(self.peers):
            self.get_logger().error("Refusing takeoff: no fresh data from every other drone. Check Wi-Fi, "
                                    "PEER_IP, and that all three scripts are running.")
        elif self.layout_ok(nbs):
            self.anchor = self.own_local_en
            self.start_z = self.own_z
            self.target_z = self.start_z - (TAKEOFF_HEIGHT + ALT_STAGGER[self.drone_id])
            self.hold_yaw = self.heading
            self.takeoff_ticks, self.last_fresh = 0, now
            self.get_logger().info(f"Takeoff OK: heading {math.degrees(self.hold_yaw):.0f} deg, climbing "
                                   f"{TAKEOFF_HEIGHT + ALT_STAGGER[self.drone_id]:.1f} m")
            self.set_phase("TAKEOFF")
            self.prompt_land()
            return
        self.prompt_takeoff()

    # ------------------------------------------------------- control law

    def compute_command(self, now, nbs):
        """Returns WORLD velocity [East, North, Down] in m/s; fills self.dbg."""
        self.dbg = {}
        if self.phase in ("WAIT_DATA", "WAIT_GO", "LAND"):
            return [0.0, 0.0, 0.0]
        if not self.own_ready(now):
            self.log_every("inv", 1.0, "warn", "Own position invalid: commanding zero velocity")
            return [0.0, 0.0, 0.0]
        vd = clamp(ALT_KP * (self.target_z - self.own_z), MAX_SPEED_Z)
        if self.phase in ("TAKEOFF", "WAIT_PEER"):
            a = clamp_vec((ANCHOR_KP * (self.anchor[0] - self.own_local_en[0]),
                           ANCHOR_KP * (self.anchor[1] - self.own_local_en[1])), ANCHOR_MAX_SPEED)
            return [a[0], a[1], vd]

        # FORMATION
        if len(nbs) < len(self.peers):
            return [0.0, 0.0, vd]                    # a neighbour is stale: hover, hold altitude
        rels = {p: self.relative_to(pkt, age) for p, (age, pkt) in nbs.items()}
        u_rep = [0.0, 0.0]
        for p, r in rels.items():
            ur = repulsion_velocity(r, D_SENSE, REP_H, 1.0 if self.drone_id > p else -1.0, AVOID_SWIRL_DEG)
            u_rep[0] += ur[0]
            u_rep[1] += ur[1]
        st = self.step
        des = slot_offset(self.drone_id, st["side"], st["swap"])
        if self.is_leader:
            # a moving target the leader chases; followers then follow the leader
            ax = self.anchor[0] + st["lvE"] * CONTROL_PERIOD_SEC
            ay = self.anchor[1] + st["lvN"] * CONTROL_PERIOD_SEC
            gx, gy = ax - self.own_local_en[0], ay - self.own_local_en[1]
            g = math.hypot(gx, gy)
            if g > MAX_ANCHOR_LEAD:
                ax, ay = self.own_local_en[0] + gx * MAX_ANCHOR_LEAD / g, self.own_local_en[1] + gy * MAX_ANCHOR_LEAD / g
                gx, gy = ax - self.own_local_en[0], ay - self.own_local_en[1]
            self.anchor = (ax, ay)
            u_form = clamp_vec((ANCHOR_KP * gx + st["lvE"], ANCHOR_KP * gy + st["lvN"]), ANCHOR_MAX_SPEED)
            rel_lead = (0.0, 0.0)
        else:
            age, lp = nbs[LEADER_ID]
            rel_lead = rels[LEADER_ID]
            u_form = formation_velocity(rel_lead, des, (self.own_ve, self.own_vn), (lp["ve"], lp["vn"]), KP, KV)
            u_form = clamp_vec((u_form[0] + FF_GAIN * lp["ve"], u_form[1] + FF_GAIN * lp["vn"]), FORMATION_MAX_SPEED)
        ue, un = clamp_vec((u_form[0] + u_rep[0], u_form[1] + u_rep[1]), MAX_SPEED_XY)
        dists = sorted(math.hypot(*r) for r in rels.values())
        self.dbg = dict(rels=rels, des=des, rel_lead=rel_lead, u_rep=math.hypot(*u_rep), dists=dists,
                        max_age=max(a for a, _ in nbs.values()))
        return [ue, un, vd]

    # ---------------------------------------------------------- loops

    def control_loop(self):
        """10 Hz: sense, decide, share, log. Publishing is done by fast_loop."""
        now = time.monotonic()
        self.tick += 1
        if now - self._last_control > 0.25 and self.tick > 5:
            self.log_every("slow", 2.0, "warn", f"control loop was late by {now - self._last_control:.2f} s")
        self._last_control = now
        nbs = self.neighbors()
        self.update_phase(now, nbs)
        cmd = self.compute_command(now, nbs)
        self._cmd = ((cmd[1], cmd[0], cmd[2]), now)       # world (E,N,D) -> PX4 NED (N,E,D)
        self.broadcast_state()
        self.write_log(now, cmd)
        if self.tick % 10 == 0 and self.dbg:
            d = self.dbg["dists"]
            self.get_logger().info(f"[{self.step['label']}] neighbours at {d[0]:.1f} m and {d[-1]:.1f} m, "
                                   f"push {self.dbg['u_rep']:.2f} m/s, heartbeat late {self.late_count}x")

    def fast_loop(self):
        """20 Hz: heartbeat + latest setpoint. Separate callback group, so a slow control tick cannot stop it."""
        now = time.monotonic()
        dt = now - self._fast_last
        self._fast_last = now
        self.fast_ticks += 1
        if dt > LATE_FAST_SEC and self.fast_ticks > 5:
            self.late_count += 1
            self.max_late = max(self.max_late, dt)
            self.log_every("late", 1.0, "warn", f"heartbeat late: {dt:.2f} s between ticks "
                           f"({self.late_count} so far, worst {self.max_late:.2f} s)")
        self.publish_offboard_control_mode()
        if self.phase in ("WAIT_DATA", "LAND"):          # no setpoints before the heading is known / after landing
            return
        cmd, t_cmd = self._cmd
        if self.phase == "WAIT_GO":
            vel = (0.0, 0.0, 0.0)
        elif now - t_cmd > CMD_FRESH_SEC:
            vel = (0.0, 0.0, 0.0)                    # control stalled: hold, do not repeat an old command
            self.log_every("stale_cmd", 1.0, "warn", "control command is stale: sending a zero-velocity hold")
        else:
            vel = cmd
        self.publish_setpoint(vel)

    def broadcast_state(self):
        e, n = self.own_world if (DEPLOYMENT == "SITL" and self.own_world) else (0.0, 0.0)
        lat, lon = self.own_latlon if self.own_latlon else (0.0, 0.0)
        st = self.step
        self.link.send(dict(id=self.drone_id, phase=self.phase, e=_f(e), n=_f(n), ve=_f(self.own_ve),
                            vn=_f(self.own_vn), lat=_f(lat), lon=_f(lon), label=st["label"], plan_idx=int(st["idx"]),
                            side=_f(st["side"]), swap=bool(st["swap"]), lvE=_f(st["lvE"]), lvN=_f(st["lvN"])))

    def write_log(self, now, cmd):
        d = self.dbg
        t = f"{now - self.t_start:.2f}"
        st = self.step
        if d:
            dist = d["dists"]
            rl, des = d["rel_lead"], d["des"]
            row = [t, self.phase, st["label"], f"{st['side']:.2f}", int(st["swap"]), f"{dist[0]:.3f}", f"{dist[-1]:.3f}",
                   f"{des[0]:.2f}", f"{des[1]:.2f}", f"{rl[0]:.3f}", f"{rl[1]:.3f}", f"{d['u_rep']:.3f}",
                   f"{cmd[0]:.3f}", f"{cmd[1]:.3f}", f"{cmd[2]:.3f}", f"{self.own_ve:.3f}", f"{self.own_vn:.3f}",
                   self.late_count, f"{d['max_age']:.3f}"]
        else:
            row = [t, self.phase, st["label"], f"{st['side']:.2f}", int(st["swap"]), "", "", "", "", "", "", "",
                   f"{cmd[0]:.3f}", f"{cmd[1]:.3f}", f"{cmd[2]:.3f}", f"{self.own_ve:.3f}", f"{self.own_vn:.3f}",
                   self.late_count, ""]
        self.csv.writerow(row)
        if self.tick % 10 == 0:
            self.csv_file.flush()

    # ------------------------------------------------------- publishing

    def publish_offboard_control_mode(self):
        m = OffboardControlMode()
        m.position, m.velocity, m.acceleration, m.attitude, m.body_rate = False, True, False, False, False
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(m)

    def publish_setpoint(self, vel_ned):
        nan3 = [float("nan")] * 3
        m = TrajectorySetpoint()
        m.position, m.acceleration, m.jerk = list(nan3), list(nan3), list(nan3)
        m.velocity = [float(v) for v in vel_ned]
        m.yaw, m.yawspeed = float(self.hold_yaw), float("nan")
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(m)

    def publish_command(self, command, param1=0.0, param2=0.0):
        m = VehicleCommand()
        m.param1, m.param2, m.command = param1, param2, command
        m.target_system = m.source_system = self.mav_sys_id
        m.target_component = m.source_component = 1
        m.from_external = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(m)

    def shutdown(self):
        """Ctrl+C while flying: don't leave PX4 waiting on a dead offboard stream."""
        if self.phase in AIRBORNE_PHASES:
            for _ in range(5):
                self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                time.sleep(0.05)
        self.link.close()
        self.csv_file.close()


def run(drone_id):
    rclpy.init()
    node = TriangleNode(drone_id)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description="Three-drone triangle formation + collision avoidance")
    p.add_argument("--id", type=int, required=True, choices=list(DRONE_IDS),
                   help="0 = leader, 1 = follower left, 2 = follower right")
    p.add_argument("--plan", choices=list(PLANS), default="full",
                   help="test schedule; only drone 0's choice matters (the others follow it). Default: full")
    args, _ = p.parse_known_args()
    global FORMATION_PLAN
    FORMATION_PLAN = PLANS[args.plan]
    run(args.id)


if __name__ == "__main__":
    main()
