#!/usr/bin/env python3
"""
Two-drone FORMATION CHANGE + COLLISION AVOIDANCE test  --  SIMULATION (PX4 SITL + Gazebo) VERSION
(sister file: two_drone_formation_change_real.py has the identical control logic)

Same control logic as your two_drone_formation_sim.py (displacement-based
formation law + Dimarogonas potential-field avoidance), but the test is now a
FORMATION CHANGE: the desired offset between the drones is changed in flight,
so the straight-line path to the new slots can run THROUGH the other drone.
Formation control and collision avoidance are then active at the same time.

DESIRED OFFSET
    D = q_1 - q_0 = (East, North) of drone 1 relative to drone 0, metres.
    Drone 1 wants rel = +D, drone 0 wants rel = -D   (rel = q_me - q_other).
    Start layout: drone 0 on the WEST side, drone 1 on the EAST side, 4-6 m
    apart, so the SETTLE step is D = (+4, 0).

TEST PLANS (--plan NAME)       D goes from (+4,0) to ...            what it tests
    rotate90       (0, +4)      path just clips the avoidance zone     mildest, do this first
    diagonal       (-3, -3)     crossing at an angle
    swap_offline   (-4, +0.5)   near head-on swap, 0.5 m off the line  which side do they pick?
    swap           (-4,  0)     dead-on head-on swap                   hardest, can stall
    swap_zone      (-1.5, 0)    swap INTO the avoidance zone           crossing + unsatisfiable slot
    pull_in        (+1.5, 0)    squeeze in through the zone            formation vs avoidance
    all            every change above, each followed by a RETURN       long: simulation only
    Every one of these = SETTLE (15 s) -> CHANGE (40 s) -> RETURN to (+4,0) (30 s).
    The RETURN is a second crossing, so each plan crosses twice.

MISSION PLANS: move in formation -> swap (cross) -> move in formation -> back to spawn
    mission          SETTLE, MOVE_NORTH (7.5 m), SWAP (stationary), MOVE_EAST (7.5 m, swapped), HOME
    mission_rotate   same, but the formation change is a 90 degree ROTATE instead of a swap (milder)
    mission_moving   same, but the SWAP happens WHILE the formation keeps moving North (hardest)
    - MOVE steps: the whole formation translates at --move-speed (default 0.3 m/s, max 0.4)
      for --move-time seconds (default 25 s = 7.5 m). Every drone adds the group velocity as a
      feed-forward on top of the formation law, so the follower does not lag behind. With a
      leader (--leader 0|1) the leader's hold point travels with the formation.
    - HOME step: each drone flies back to ITS OWN takeoff spot (the spawn spot in the sim) with
      the avoidance still on. After the swap the drones are on opposite sides, so going home is
      another crossing. The plan then lands them at home.
    - Space needed on the field: roughly 15 m x 12 m clear for mission / mission_rotate (7.5 m North,
      then 7.5 m East, drones 4 m apart); mission_moving moves further North (about 9 m more), so
      allow about 25 m x 12 m. Shorten with --move-time if the area is small.
    - Pass: dist stays above the critical radius in every step; err_form is ~0 at the end of
      each FORM step (look at it during MOVE too: it should stay small); at the end of HOME
      anchor_err is near 0 (they are back at their spawn spots).

ROLES (--leader 0|1|none)
    0 or 1  : that drone holds its takeoff spot and the other one moves (directed).
    none    : BOTH drones move toward the new slots and meet head-on.
              This is the tightest case in the model. Use it after the directed runs.
    The master (drone 0) owns the schedule and the role and broadcasts them,
    so only drone 0's --plan / --leader matter.

COMPARING YOUR DISPLACEMENT LAW ALONE vs WITH THE GROUP FEED-FORWARD
    Your law only holds the SHAPE (relative offsets); it has no term that moves the whole group.
    The MOVE steps get their motion from a group velocity that the master broadcasts.
      --ff on  (default)  followers add the group velocity: they move with the leader, no lag.
      --ff off            followers use the displacement law ALONE; only the leader moves (needs
                          --leader 0|1). The follower then trails by about speed/KP (0.75 m at
                          0.3 m/s with KP 0.4) and starts moving only after an error builds up.
      --kp 1 --kv 0       exactly your first-order law  u2 = -(P2-P1) - (P1_des-P2_des)
                          (the default KP 0.4 / KV 0.5 is the PD form, survey Eq. 17).
    Run the same plan twice (--ff on, then --ff off), keep the two CSVs from the FOLLOWER and compare:
        python3 compare_runs.py logA.csv logB.csv --labels "FF on" "FF off"
    Only drone 0's --ff / --leader are used (the master broadcasts them); set --kp/--kv on both terminals.

PASS CRITERIA (read them from the CSV)
    - dist_true (SITL) or dist_h (real) never below the critical radius
      D_SENSE/sqrt(3) = 1.73 m. Margin matters more than the exact number.
    - err_form returns to ~0 after each step, except the *_zone plans where the
      avoidance deliberately holds the drones apart (err_form stays about 1.5 m).
      (In a HOME step err_form is meaningless: look at anchor_err instead.)
    - No chattering: u_rep_e / u_rep_n should not flip sign every sample.
    - The script prints the closest approach at each step and when it exits.

SITL SETUP
    - SPAWN_WORLD_OFFSET MUST equal each instance's PX4_GZ_MODEL_POSE (East, North).
      Default here: drone 0 at (0, 0), drone 1 at (5, 0).
    - Instance -i 0 -> topics /fmu/..., instance -i 1 -> /px4_1/fmu/...
    - One Micro XRCE-DDS Agent for both. Check the topic names against
      `ros2 topic list` (your SITL status topic was vehicle_status_v4).
    - SITL-only extras to stress the controller:
        --sep-error 1.0   each drone believes the other is 1 m FARTHER than it is
                          (the dangerous direction: they get closer than they think)
        --noise 0.3       Gaussian noise on the measured relative position (m)
        --drop 0.2        drop 20 % of received UDP packets
        --swirl 0         the pure paper law (no sidestep) to see the head-on stall

REAL SETUP
    - One Pi per drone. `hostname -I` on each Pi, same Wi-Fi.
      Pass the OTHER drone's IP:   --peer-ip 192.168.x.x
    - Check each drone's real MAV_SYS_ID (`param show MAV_SYS_ID`) against
      REAL_MAV_SYS_ID below. A wrong value = that drone ignores arm/mode/land.
    - Default height stagger is drone 1 = +1.0 m. The avoidance only looks at
      HORIZONTAL distance, so the crossing is still a full test but a bug cannot put
      the drones in the same airspace. Remove it only with an explicit --stagger 0,
      and only after the staggered runs look good. Downwash from the upper drone can
      push the lower one: use more than 1 m if your test ceiling allows.
    - The stagger is set on drone 1's terminal (drone 1 flies the extra height).
    - The injection options above are refused in REAL mode.

USAGE (one terminal per drone, on that drone's Pi / or both on the laptop in SITL)
    python3 <this file> --id 0 --plan mission --leader 0
    python3 <this file> --id 1 --plan mission --leader 0
    optional: --move-speed 0.2 --move-time 20   (shorter, slower moves for the first real tries)
    - When a drone says "Press ENTER to ARM + TAKE OFF", press ENTER in both.
    - ENTER again at any time lands that drone (the other one lands too).
    - Both Pis must run the SAME version of this file (the packet format changed).

SAFETY
    - Props OFF for the first run: check messages, link and layout.
    - RC transmitter in hand for BOTH drones. This script is not a failsafe.
    - Take over with the RC MODE switch (not the arm switch). After a takeover,
      stop the script: Ctrl+C sends a land command.
    - Link lost for LINK_LOST_LAND_SEC, or either drone landing -> both land.
    - Yaw is held at each drone's starting heading.

LOG
    formation_change_log_drone<ID>_<time>.csv in the current folder (10 Hz).
"""

import argparse
import csv
import json
import math
import random
import socket
import threading
import time

import rclpy
from rclpy.node import Node
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

DEPLOYMENT = "SITL"   # fixed for this file; two_drone_formation_change_real.py is the hardware version

# --- Topics. Check with `ros2 topic list | grep fmu/out` and edit to match ---
LOCAL_POSITION_TOPIC = "vehicle_local_position_v1"
GLOBAL_POSITION_TOPIC = "vehicle_global_position"
VEHICLE_STATUS_TOPIC = "vehicle_status_v1" if DEPLOYMENT == "REAL" else "vehicle_status_v4"
REAL_NAMESPACE = ""   # each Pi sees only its own drone: bare /fmu/...

# --- REAL: each vehicle's actual MAV_SYS_ID (`param show MAV_SYS_ID`) ---
# VERIFY BOTH. Wrong value = that drone silently ignores arm/mode/land commands.
REAL_MAV_SYS_ID = {0: 1, 1: 2}

# --- UDP link between the two drones ---
# REAL: pass the other drone's IP with --peer-ip, or put the IPs here.
# SITL: both on localhost, the defaults work as they are.
PEER_IP = {0: "CHANGE_ME", 1: "CHANGE_ME"} if DEPLOYMENT == "REAL" else {0: "127.0.0.1", 1: "127.0.0.1"}
BASE_PORT = 14600            # drone N listens on BASE_PORT + N
NEIGHBOR_TIMEOUT_SEC = 0.5   # packet older than this = neighbor data is stale
LINK_LOST_LAND_SEC = 5.0     # stale this long while flying -> land
MAX_EXTRAPOLATE_SEC = 0.5    # predict neighbor position forward by v*age, capped

# --- SITL only: spawn positions in the shared world (X=East, Y=North), metres.
#     Must match PX4_GZ_MODEL_POSE for each instance. id 0 must be WEST of id 1.
SPAWN_WORLD_OFFSET = {0: (0.0, 0.0), 1: (5.0, 0.0)}

PEER_ID = {0: 1, 1: 0}
MASTER_ID = 0          # owns the test schedule and the role
LEADER_ID = 0          # holds position; None = both drones move (symmetric)

# --- Test plans. Each step = (label, D_east, D_north, g_east, g_north, hold seconds, mode)
#   D = q_1 - q_0 : the desired offset of drone 1 from drone 0 (the formation SHAPE)
#   g = group velocity (m/s): the whole formation TRANSLATES at this speed (0 = stationary)
#   mode "FORM" = formation law + avoidance (+ group velocity);  "HOME" = every drone flies
#   back to ITS OWN takeoff spot (still with avoidance on) = "return to spawn".
SETTLE_D = (4.0, 0.0)
_CHANGES = [
    ("rotate90",     "ROTATE90",       (0.0, 4.0)),
    ("diagonal",     "DIAGONAL",       (-3.0, -3.0)),
    ("swap_offline", "SWAP_OFFLINE",   (-4.0, 0.5)),
    ("swap",         "SWAP",           (-4.0, 0.0)),
    ("swap_zone",    "SWAP_INTO_ZONE", (-1.5, 0.0)),
    ("pull_in",      "PULL_IN",        (1.5, 0.0)),
]
SETTLE_HOLD, CHANGE_HOLD, RETURN_HOLD = 15.0, 40.0, 30.0
MOVE_SPEED = 0.3      # m/s group speed in the MOVE steps (max 0.4; start at 0.3 or less)
MOVE_SECONDS = 25.0   # 0.3 m/s * 25 s = 7.5 m per move
HOME_HOLD = 50.0      # minimum time allowed to get back to the spawn spots (longer plans get more)
MOVE_SPEED_MAX = 0.4


def _step(label, d, g, hold, mode="FORM"):
    return (label, d[0], d[1], g[0], g[1], hold, mode)


def _build_plans(move_speed=None, move_seconds=None):
    ms = MOVE_SPEED if move_speed is None else move_speed
    mt = MOVE_SECONDS if move_seconds is None else move_seconds
    Z = (0.0, 0.0)
    plans = {}
    for key, label, d in _CHANGES:
        plans[key] = [_step("SETTLE", SETTLE_D, Z, SETTLE_HOLD),
                      _step(label, d, Z, CHANGE_HOLD),
                      _step("RETURN", SETTLE_D, Z, RETURN_HOLD)]
    longest = [_step("SETTLE", SETTLE_D, Z, SETTLE_HOLD)]
    for key, label, d in _CHANGES[:-1]:        # all but pull_in
        longest += [_step(label, d, Z, CHANGE_HOLD), _step("RETURN", SETTLE_D, Z, RETURN_HOLD)]
    plans["all"] = longest
    # --- missions: move in formation -> swap (cross) -> move in formation -> back to spawn ---
    for key, label, d, swap_moving in (("mission", "SWAP", (-4.0, 0.0), False),
                                       ("mission_rotate", "ROTATE90", (0.0, 4.0), False),
                                       ("mission_moving", "SWAP_MOVING", (-4.0, 0.0), True)):
        swap_hold = 30.0 if swap_moving else CHANGE_HOLD
        north = ms * mt + (ms * swap_hold if swap_moving else 0.0)     # how far the formation ends up
        east = ms * mt                                                 # from the spawn spots
        home_hold = max(HOME_HOLD, math.hypot(north, east) / 0.35 + 25.0)   # time to fly back at <= 0.5 m/s
        plans[key] = [_step("SETTLE", SETTLE_D, Z, SETTLE_HOLD),
                      _step("MOVE_NORTH", SETTLE_D, (0.0, ms), mt),
                      _step(label, d, (0.0, ms) if swap_moving else Z, swap_hold),
                      _step("MOVE_EAST", d, (ms, 0.0), mt),
                      _step("HOME", d, Z, home_hold, "HOME")]
    return plans


PLANS = _build_plans()
PLAN_NAME = "rotate90"
FORMATION_PLAN = PLANS[PLAN_NAME]
AUTO_LAND = True        # land after the last step; False = hover until ENTER
GROUP_FEEDFORWARD = True   # True : followers also add the group velocity (no lag behind a moving formation)
                           # False: followers use the displacement law ALONE; only the leader moves, so the
                           #        follower trails by about speed/KP (needs a leader; --ff off)

# --- Mission geometry ---
TAKEOFF_HEIGHT = 2.0    # m above the spot each drone starts on
# extra metres per drone. Real default: drone 1 flies 1 m higher (see docstring).
ALT_STAGGER = {0: 0.0, 1: 1.0} if DEPLOYMENT == "REAL" else {0: 0.0, 1: 0.0}
ARRIVAL_THRESHOLD = 0.3
ALLOW_ANY_START_LAYOUT = False   # this test needs drone 0 WEST of drone 1
MIN_START_SEPARATION = 3.0       # m: refuse takeoff if closer than this at the start
MAX_START_SEPARATION = 20.0      # m: refuse if farther (catches GPS/frame mistakes)

# --- Formation gains (PD, survey Eq. 17). Raise KV first if it oscillates ---
KP = 0.4
KV = 0.5
FORMATION_MAX_SPEED = 0.5   # cap on the formation pull BEFORE avoidance is added. Must stay below
                            # REPULSION_SPEED_AT_CRITICAL, or a big formation error can push a
                            # drone through the safety radius.
MAX_SPEED_XY = 0.8      # m/s horizontal command cap
MAX_SPEED_Z = 0.6       # m/s vertical command cap
ALT_KP = 0.5
ANCHOR_KP = 0.5         # position hold on the takeoff spot (leader / takeoff / wait)
ANCHOR_MAX_SPEED = 0.5

# --- Collision avoidance potential (Dimarogonas 2006) ---
D_SENSE = 3.0
REPULSION_SPEED_AT_CRITICAL = 1.0   # m/s of push-away at the critical radius
AVOID_SWIRL_DEG = 30.0              # extra to the paper, say so in the report (--swirl 0 = pure paper)
assert FORMATION_MAX_SPEED < REPULSION_SPEED_AT_CRITICAL, "avoidance must out-push the formation at the critical radius"
_C = D_SENSE ** 2 / 3.0                                   # paper: d^2 = 3c
REP_H = REPULSION_SPEED_AT_CRITICAL / (8.0 * _C ** 1.5)   # paper: a = 4*h*c^3
CRIT_RADIUS = D_SENSE / math.sqrt(3.0)

# --- SITL-only stress options (refused in REAL). Also settable from the command line. ---
SIM_SEP_ERROR = 0.0        # m: each drone thinks the other is this much FARTHER (>0 = dangerous)
SIM_REL_NOISE_STD = 0.0    # m: Gaussian noise on the measured relative position
SIM_PACKET_DROP = 0.0      # 0..1: probability of dropping a received packet

# --- Timeouts ---
TAKEOFF_TIMEOUT_SEC = 25.0
WAIT_PEER_TIMEOUT_SEC = 60.0
CONTROL_PERIOD_SEC = 0.1   # 10 Hz (PX4 needs >= 2 Hz offboard stream)

AIRBORNE_PHASES = ("TAKEOFF", "WAIT_PEER", "FORMATION")

# ---------------------------------------------------------------------------
# PURE MATH (no ROS): potential field, formation law, geometry
# ---------------------------------------------------------------------------


def ll_to_en_diff(lat_a, lon_a, lat_b, lon_b):
    """(east, north) of A relative to B in metres. Flat-earth, fine at tens of metres."""
    R = 6371000.0
    north = math.radians(lat_a - lat_b) * R
    east = math.radians(lon_a - lon_b) * R * math.cos(math.radians((lat_a + lat_b) / 2.0))
    return east, north


def clamp(x, lim):
    return max(-lim, min(lim, x))


def clamp_vec(v, vmax):
    n = math.hypot(v[0], v[1])
    if n > vmax and n > 0.0:
        s = vmax / n
        return (v[0] * s, v[1] * s)
    return (v[0], v[1])


def potential_rho(beta, d, h):
    """
    rho = dV/dbeta for V(beta), beta = ||q_i - q_j||^2  (paper, Section III):
        V = a/beta                 beta < c
        V = h (beta - d^2)^2       c <= beta < d^2
        V = 0                      beta >= d^2          with d^2 = 3c, a = 4 h c^3
    """
    c = d * d / 3.0
    if beta >= d * d:
        return 0.0
    if beta >= c:
        return 2.0 * h * (beta - d * d)
    a = 4.0 * h * c ** 3
    return -a / (beta * beta)


def repulsion_velocity(rel, d, h, fallback_sign, swirl_deg=0.0):
    """
    -dV_ij/dq_i = -2 rho (q_i - q_j): pushes drone i directly away from j.
    rel = q_i - q_j (East, North). Zero outside the sensing radius d.
    """
    e, n = rel
    if math.hypot(e, n) < 0.05:          # (almost) on top of each other: pick a side
        e, n = fallback_sign * 0.05, 0.0
    beta = max(e * e + n * n, 1e-6)
    k = -2.0 * potential_rho(beta, d, h)
    ue, un = k * e, k * n
    if swirl_deg:                        # rotate counter-clockwise: each drone sidesteps to ITS right
        a = math.radians(swirl_deg)
        ue, un = ue * math.cos(a) - un * math.sin(a), ue * math.sin(a) + un * math.cos(a)
    return (ue, un)


def formation_velocity(rel, des, v_own, v_nb, kp, kv):
    """u_i = -kp*((q_i - q_j) - (q_i* - q_j*)) - kv*(v_i - v_j)"""
    return (-kp * (rel[0] - des[0]) - kv * (v_own[0] - v_nb[0]),
            -kp * (rel[1] - des[1]) - kv * (v_own[1] - v_nb[1]))


# ---------------------------------------------------------------------------
# UDP LINK between the drones
# ---------------------------------------------------------------------------

_REQUIRED_KEYS = ("id", "phase", "valid", "e", "n", "ve", "vn", "lat", "lon",
                  "des_e", "des_n", "g_e", "g_n", "m_e", "m_n", "mode", "ff", "leader", "plan_idx", "label")
_FLOAT_KEYS = ("e", "n", "ve", "vn", "lat", "lon", "des_e", "des_n", "g_e", "g_n", "m_e", "m_n")


class PeerLink:
    def __init__(self, my_id, listen_port, peer_addrs, drop=0.0):
        self.my_id = my_id
        self.peer_addrs = peer_addrs
        self.drop = drop               # SITL stress option; always 0 in REAL
        self.latest = {}               # sender id -> (monotonic receive time, packet)
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
                if pkt["id"] == self.my_id:
                    continue
                for k in _FLOAT_KEYS:
                    if not isinstance(pkt[k], (int, float)) or not math.isfinite(pkt[k]):
                        raise ValueError(k)
                if not isinstance(pkt["leader"], int) or pkt["leader"] not in (-1, 0, 1):
                    raise ValueError("leader")
                if pkt["mode"] not in ("FORM", "HOME"):
                    raise ValueError("mode")
                if isinstance(pkt["ff"], bool) or pkt["ff"] not in (0, 1):
                    raise ValueError("ff")
            except (ValueError, UnicodeDecodeError):
                continue
            if self.drop > 0.0 and random.random() < self.drop:
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


# ---------------------------------------------------------------------------
# NODE
# ---------------------------------------------------------------------------


class TwoDroneFormationNode(Node):
    def __init__(self, drone_id):
        super().__init__(f"two_drone_formation_change_{drone_id}")
        self.drone_id = drone_id
        self.peer_id = PEER_ID[drone_id]
        self.side = -1.0 if drone_id == 0 else 1.0     # -1 = West/left, +1 = East/right
        self.is_master = drone_id == MASTER_ID
        self.leader_id = LEADER_ID                      # non-master adopts the master's value
        self.namespace = get_namespace(drone_id)
        self.mav_sys_id = REAL_MAV_SYS_ID[drone_id] if DEPLOYMENT == "REAL" else drone_id + 1

        if DEPLOYMENT == "REAL" and PEER_IP[self.peer_id] == "CHANGE_ME":
            raise RuntimeError("Pass the other drone's IP with --peer-ip (hostname -I on its Pi).")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        ns = self.namespace
        self.offboard_pub = self.create_publisher(OffboardControlMode, f"{ns}/fmu/in/offboard_control_mode", qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, f"{ns}/fmu/in/trajectory_setpoint", qos)
        self.command_pub = self.create_publisher(VehicleCommand, f"{ns}/fmu/in/vehicle_command", qos)
        self.create_subscription(VehicleLocalPosition, f"{ns}/fmu/out/{LOCAL_POSITION_TOPIC}",
                                 self.local_position_cb, qos)
        self.create_subscription(VehicleStatus, f"{ns}/fmu/out/{VEHICLE_STATUS_TOPIC}",
                                 self.status_cb, qos)
        if DEPLOYMENT == "REAL":
            self.create_subscription(VehicleGlobalPosition, f"{ns}/fmu/out/{GLOBAL_POSITION_TOPIC}",
                                     self.global_position_cb, qos)

        self.link = PeerLink(
            my_id=drone_id,
            listen_port=BASE_PORT + drone_id,
            peer_addrs=[(PEER_IP[self.peer_id], BASE_PORT + self.peer_id)],
            drop=SIM_PACKET_DROP if DEPLOYMENT == "SITL" else 0.0,
        )

        # --- own state ---
        self.local_valid = False
        self.last_local_time = -1e9
        self.own_local_en = (0.0, 0.0)    # (East, North) in PX4 local frame (only for anchor hold)
        self.own_world = None             # SITL: (E, N) in shared world frame
        self.own_latlon = None            # REAL: (lat, lon)
        self.own_ve = self.own_vn = self.own_vd = 0.0
        self.own_z = 0.0                  # NED down. Never compared between drones.
        self.heading = None
        self.vehicle_status = VehicleStatus()

        # --- mission state ---
        self.phase = "WAIT_DATA"
        self.phase_t0 = time.monotonic()
        self.go = False
        self.land_requested = False
        self.anchor = (0.0, 0.0)
        self.start_z = 0.0
        self.target_z = 0.0
        self.hold_yaw = 0.0
        self.takeoff_ticks = 0
        self.last_fresh = time.monotonic()
        self.plan_idx = 0
        self.plan_t0 = time.monotonic()
        self.plan_done = False
        self.des_e, self.des_n = FORMATION_PLAN[0][1], FORMATION_PLAN[0][2]   # shared D = q_1 - q_0
        self.plan_label = FORMATION_PLAN[0][0]
        self.g_e, self.g_n = 0.0, 0.0     # group velocity (formation translation), m/s
        self.m_e, self.m_n = 0.0, 0.0     # translation accumulated since the plan started (master integrates it)
        self.step_mode = "FORM"
        self.ff = GROUP_FEEDFORWARD       # master owns it; non-master adopts the master's value
        self.last_m_t = time.monotonic()
        self.min_dist = float("inf")
        self.land_cmd_t0 = 0.0
        self.last_land_resend = 0.0
        self._last_log = {}
        self.dbg = {}
        self.tick = 0

        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_file = open(f"formation_change_log_drone{drone_id}_{stamp}.csv", "w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(["t_s", "phase", "label", "mode", "D_e", "D_n", "g_e", "g_n", "m_e", "m_n",
                           "anchor_err", "dist_want", "dist_h", "dist_true",
                           "err_form", "below_crit", "rel_e", "rel_n",
                           "u_form_e", "u_form_n", "u_rep_e", "u_rep_n", "cmd_e", "cmd_n", "cmd_d",
                           "own_ve", "own_vn", "nb_age", "ff"])
        self.t_start = time.monotonic()

        self.timer = self.create_timer(CONTROL_PERIOD_SEC, self.control_loop)
        total = sum(step[5] for step in FORMATION_PLAN)
        self.get_logger().info(
            f"Drone {drone_id} ({'LEFT/West' if self.side < 0 else 'RIGHT/East'}) started: "
            f"{DEPLOYMENT}, mav_sys_id={self.mav_sys_id}, leader={self.leader_id}, master={self.is_master}, "
            f"plan={PLAN_NAME} ({len(FORMATION_PLAN)} steps, {total:.0f} s), "
            f"D_SENSE={D_SENSE} m, critical radius={CRIT_RADIUS:.2f} m, swirl={AVOID_SWIRL_DEG:.0f} deg, "
            f"KP={KP}, KV={KV}, group feed-forward={'ON' if GROUP_FEEDFORWARD else 'OFF (law alone)'}")
        if DEPLOYMENT == "REAL":
            if ALT_STAGGER[0] == ALT_STAGGER[1]:
                self.get_logger().warn("NO HEIGHT STAGGER: the drones fly at the same height. "
                                       "Only do this after staggered runs looked good.")
            else:
                self.get_logger().info(f"Height stagger: drone 1 flies {ALT_STAGGER[1] - ALT_STAGGER[0]:.1f} m higher.")
        else:
            if SIM_SEP_ERROR or SIM_REL_NOISE_STD or SIM_PACKET_DROP:
                self.get_logger().warn(f"SITL stress options ON: sep-error={SIM_SEP_ERROR} m, "
                                       f"noise={SIM_REL_NOISE_STD} m, drop={SIM_PACKET_DROP}")

    @property
    def is_leader(self):
        return self.leader_id is not None and self.drone_id == self.leader_id

    # ----------------------------------------------------------------- utils

    def log_every(self, key, period, level, msg):
        now = time.monotonic()
        if now - self._last_log.get(key, -1e9) >= period:
            self._last_log[key] = now
            getattr(self.get_logger(), level)(msg)

    def set_phase(self, new_phase):
        self.get_logger().info(f"Phase: {self.phase} -> {new_phase}")
        self.phase = new_phase
        self.phase_t0 = time.monotonic()

    # ------------------------------------------------------------- callbacks

    def local_position_cb(self, msg):
        self.last_local_time = time.monotonic()
        self.local_valid = bool(msg.xy_valid and msg.z_valid)
        if not self.local_valid:
            return
        # PX4 local NED is (x=North, y=East); we use (East, North).
        self.own_local_en = (msg.y, msg.x)
        self.own_ve, self.own_vn, self.own_vd = msg.vy, msg.vx, msg.vz
        self.own_z = msg.z
        if math.isfinite(msg.heading):
            self.heading = msg.heading
        if DEPLOYMENT == "SITL":
            sx, sy = SPAWN_WORLD_OFFSET[self.drone_id]
            self.own_world = (sx + msg.y, sy + msg.x)

    def global_position_cb(self, msg):
        if msg.lat_lon_valid:
            self.own_latlon = (msg.lat, msg.lon)
        else:
            self.own_latlon = None

    def status_cb(self, msg):
        self.vehicle_status = msg

    # ------------------------------------------------------------ data state

    def own_ready(self, now):
        if not self.local_valid or self.heading is None or now - self.last_local_time > 1.0:
            return False
        if DEPLOYMENT == "REAL":
            return self.own_latlon is not None
        return self.own_world is not None

    def neighbor(self, now):
        """(age, packet) if fresh AND the neighbor reports a valid position, else None."""
        item = self.link.get(self.peer_id)
        if item is None or item[0] > NEIGHBOR_TIMEOUT_SEC:
            return None
        if not item[1].get("valid"):
            return None
        return item

    def relative_to(self, pkt, age):
        """q_i - q_j (East, North) in metres, neighbor predicted forward by v*age."""
        if DEPLOYMENT == "REAL":
            e, n = ll_to_en_diff(self.own_latlon[0], self.own_latlon[1], pkt["lat"], pkt["lon"])
        else:
            e = self.own_world[0] - pkt["e"]
            n = self.own_world[1] - pkt["n"]
        age = min(age, MAX_EXTRAPOLATE_SEC)
        e, n = e - pkt["ve"] * age, n - pkt["vn"] * age
        if DEPLOYMENT == "SITL":
            e, n = self.apply_sim_errors(e, n)
        return (e, n)

    @staticmethod
    def apply_sim_errors(e, n):
        """SITL stress: make the measured relative position wrong on purpose."""
        d = math.hypot(e, n)
        if SIM_SEP_ERROR and d > 0.05:
            e += SIM_SEP_ERROR * e / d        # >0: the other drone looks farther away than it is
            n += SIM_SEP_ERROR * n / d
        if SIM_REL_NOISE_STD > 0.0:
            e += random.gauss(0.0, SIM_REL_NOISE_STD)
            n += random.gauss(0.0, SIM_REL_NOISE_STD)
        return e, n

    def true_distance(self, pkt):
        """SITL only: real horizontal distance from the shared world frame (no errors)."""
        if DEPLOYMENT != "SITL" or self.own_world is None:
            return None
        return math.hypot(self.own_world[0] - pkt["e"], self.own_world[1] - pkt["n"])

    # ------------------------------------------------------- ENTER prompts

    def start_takeoff_prompt(self):
        def worker():
            try:
                input("\n>>> Both drones in place? Press ENTER to ARM + TAKE OFF: ")
                self.go = True
            except (EOFError, OSError):
                self.get_logger().error("No terminal attached: cannot ask for ENTER, so it will not take off.")
        threading.Thread(target=worker, daemon=True).start()

    def start_land_prompt(self):
        def worker():
            try:
                input("\n>>> Press ENTER at any time to LAND: ")
                self.land_requested = True
            except (EOFError, OSError):
                pass
        threading.Thread(target=worker, daemon=True).start()

    # ------------------------------------------------------ phase machine

    def start_landing(self, reason):
        self.get_logger().warn(f"LANDING: {reason}")
        self.report_min_dist()
        self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.land_cmd_t0 = self.last_land_resend = time.monotonic()
        self.set_phase("LAND")

    def report_min_dist(self):
        if math.isfinite(self.min_dist):
            tag = "BELOW critical radius!" if self.min_dist < CRIT_RADIUS else "above critical radius"
            self.get_logger().info(f"Closest approach so far: {self.min_dist:.2f} m ({tag}, critical {CRIT_RADIUS:.2f} m)")

    def update_phase(self, now, nb):
        ph = self.phase

        if ph == "WAIT_DATA":
            if self.own_ready(now):
                self.hold_yaw = self.heading
                self.set_phase("WAIT_GO")
                self.start_takeoff_prompt()
            else:
                self.log_every("wait_data", 3.0, "info", "Waiting for a valid position/heading from PX4 "
                               "(GPS lock outdoors, EKF ready)...")
            return

        if ph == "WAIT_GO":
            self.hold_yaw = self.heading if self.heading is not None else self.hold_yaw
            if self.go:
                self.go = False                 # consumed; a refusal needs a fresh ENTER
                self.try_start_takeoff(now, nb)
            return

        if ph == "LAND":
            # Resend the land command for a few seconds in case a BEST_EFFORT packet was lost.
            if now - self.land_cmd_t0 < 10.0 and now - self.last_land_resend >= 1.0:
                self.last_land_resend = now
                self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            return

        # ---- airborne phases: common safety rules ----
        if nb is not None:
            self.last_fresh = now
            if nb[1]["phase"] == "LAND":
                self.start_landing("the other drone is landing")
                return
        elif now - self.last_fresh > LINK_LOST_LAND_SEC:
            self.start_landing(f"no data from the other drone for {LINK_LOST_LAND_SEC:.0f} s")
            return
        else:
            self.log_every("stale", 1.0, "warn", "Neighbor data is stale: holding position")
        if self.land_requested:
            self.start_landing("ENTER pressed")
            return
        if self.takeoff_ticks >= 30 and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.log_every("offboard", 5.0, "warn", "PX4 is not in Offboard mode (RC takeover, failsafe, or "
                           "status topic not arriving?)")

        if ph == "TAKEOFF":
            self.takeoff_ticks += 1
            if self.takeoff_ticks == 10:
                self.engage_offboard()
                self.arm()
            if abs(self.own_z - self.target_z) < ARRIVAL_THRESHOLD and self.takeoff_ticks > 20:
                self.set_phase("WAIT_PEER")
            elif now - self.phase_t0 > TAKEOFF_TIMEOUT_SEC:
                self.start_landing("takeoff timed out (did the drone arm? check QGC messages)")

        elif ph == "WAIT_PEER":
            if nb is not None and nb[1]["phase"] in ("WAIT_PEER", "FORMATION"):
                self.plan_idx, self.plan_t0, self.plan_done = 0, now, False
                self.m_e = self.m_n = 0.0
                self.last_m_t = now
                self.set_phase("FORMATION")
            elif now - self.phase_t0 > WAIT_PEER_TIMEOUT_SEC:
                self.start_landing("the other drone never became ready")

        elif ph == "FORMATION":
            if self.is_master:
                label, de, dn, ge, gn, hold, mode = FORMATION_PLAN[self.plan_idx]
                self.des_e, self.des_n, self.plan_label = de, dn, label
                self.g_e, self.g_n, self.step_mode = ge, gn, mode
                dt = min(max(now - self.last_m_t, 0.0), 0.3)     # integrate the group translation
                self.last_m_t = now
                self.m_e += ge * dt
                self.m_n += gn * dt
                if not self.plan_done and now - self.plan_t0 >= hold:
                    self.report_min_dist()
                    if self.plan_idx + 1 < len(FORMATION_PLAN):
                        self.plan_idx += 1
                        self.plan_t0 = now
                        nxt = FORMATION_PLAN[self.plan_idx]
                        self.get_logger().info(
                            f"Plan step -> {nxt[0]} ({nxt[6]}): D = ({nxt[1]:+.1f}, {nxt[2]:+.1f}) m, "
                            f"group speed ({nxt[3]:+.2f}, {nxt[4]:+.2f}) m/s, {nxt[5]:.0f} s")
                    else:
                        self.plan_done = True
                        if AUTO_LAND:
                            self.start_landing("test plan finished")
                        else:
                            self.get_logger().info("Plan finished: holding. ENTER to land.")
            elif nb is not None and self.peer_id == MASTER_ID:
                pkt = nb[1]
                if pkt["label"] != self.plan_label:
                    self.get_logger().info(f"Plan step -> {pkt['label']} ({pkt['mode']})")
                self.des_e, self.des_n = pkt["des_e"], pkt["des_n"]
                self.g_e, self.g_n, self.m_e, self.m_n = pkt["g_e"], pkt["g_n"], pkt["m_e"], pkt["m_n"]
                self.step_mode = pkt["mode"]
                self.ff = bool(pkt["ff"])
                self.plan_label, self.plan_idx = pkt["label"], pkt["plan_idx"]
                self.leader_id = None if pkt["leader"] < 0 else pkt["leader"]

    def try_start_takeoff(self, now, nb):
        if not self.own_ready(now):
            self.get_logger().error("Refusing takeoff: own position/heading not valid.")
            self.start_takeoff_prompt()
            return
        if nb is None:
            self.get_logger().error("Refusing takeoff: no fresh, valid data from the other drone. "
                                    "Check Wi-Fi, --peer-ip, and that the other script is running and has a position.")
            self.start_takeoff_prompt()
            return
        age, pkt = nb
        rel = self.relative_to(pkt, age)
        sep = math.hypot(rel[0], rel[1])
        if not ALLOW_ANY_START_LAYOUT and rel[0] * self.side <= 0.0:
            self.get_logger().error(f"Refusing takeoff: drone 0 must start WEST of drone 1 "
                                    f"(this drone is on the wrong side, rel East = {rel[0]:.1f} m).")
        elif not (MIN_START_SEPARATION <= sep <= MAX_START_SEPARATION):
            self.get_logger().error(f"Refusing takeoff: separation {sep:.1f} m is outside "
                                    f"[{MIN_START_SEPARATION}, {MAX_START_SEPARATION}] m.")
        else:
            self.anchor = self.own_local_en
            self.start_z = self.own_z
            self.target_z = self.start_z - (TAKEOFF_HEIGHT + ALT_STAGGER[self.drone_id])
            self.hold_yaw = self.heading
            self.takeoff_ticks = 0
            self.last_fresh = now
            self.get_logger().info(
                f"Takeoff OK: separation {sep:.1f} m, heading {math.degrees(self.hold_yaw):.0f} deg, "
                f"climbing {TAKEOFF_HEIGHT + ALT_STAGGER[self.drone_id]:.1f} m")
            self.set_phase("TAKEOFF")
            self.start_land_prompt()
            return
        self.start_takeoff_prompt()

    # ----------------------------------------------------------- the control law

    def compute_command(self, now, nb):
        """Returns WORLD velocity command [East, North, Down] in m/s."""
        self.dbg = {}
        if self.phase in ("WAIT_DATA", "WAIT_GO", "LAND"):
            return [0.0, 0.0, 0.0]
        if not self.own_ready(now):
            self.log_every("own_invalid", 1.0, "warn", "Own position invalid: commanding zero velocity")
            return [0.0, 0.0, 0.0]

        vd = clamp(ALT_KP * (self.target_z - self.own_z), MAX_SPEED_Z)
        anchor_cmd = clamp_vec((ANCHOR_KP * (self.anchor[0] - self.own_local_en[0]),
                                ANCHOR_KP * (self.anchor[1] - self.own_local_en[1])), ANCHOR_MAX_SPEED)

        if self.phase in ("TAKEOFF", "WAIT_PEER"):
            return [anchor_cmd[0], anchor_cmd[1], vd]

        # FORMATION
        if nb is None:
            return [0.0, 0.0, vd]            # stale neighbor: hover, hold altitude
        age, pkt = nb
        rel = self.relative_to(pkt, age)
        des = (self.side * self.des_e, self.side * self.des_n)
        u_rep = repulsion_velocity(rel, D_SENSE, REP_H, self.side, AVOID_SWIRL_DEG)
        g = (self.g_e, self.g_n)
        if self.step_mode == "HOME":
            # each drone flies back to ITS OWN takeoff spot; the avoidance stays on top
            u_form = anchor_cmd
        elif self.is_leader:
            # the leader's hold point travels with the formation: takeoff spot + translation so far
            ref_e, ref_n = self.anchor[0] + self.m_e, self.anchor[1] + self.m_n
            lead = clamp_vec((ANCHOR_KP * (ref_e - self.own_local_en[0]),
                              ANCHOR_KP * (ref_n - self.own_local_en[1])), ANCHOR_MAX_SPEED)
            u_form = (lead[0] + g[0], lead[1] + g[1])
        else:
            u_form = clamp_vec(formation_velocity(rel, des, (self.own_ve, self.own_vn),
                                                  (pkt["ve"], pkt["vn"]), KP, KV), FORMATION_MAX_SPEED)
            if self.ff:                                         # feed-forward: no lag behind a moving formation
                u_form = (u_form[0] + g[0], u_form[1] + g[1])
        ue, un = clamp_vec((u_form[0] + u_rep[0], u_form[1] + u_rep[1]), MAX_SPEED_XY)

        true_d = self.true_distance(pkt)
        d_check = true_d if true_d is not None else math.hypot(*rel)
        self.min_dist = min(self.min_dist, d_check)
        if d_check < CRIT_RADIUS:
            self.log_every("crit", 1.0, "warn", f"CLOSER than the critical radius: {d_check:.2f} m < {CRIT_RADIUS:.2f} m")

        self.dbg = dict(rel=rel, des=des, u_form=u_form, u_rep=u_rep, age=age, true_d=true_d,
                        anchor_err=math.hypot(self.anchor[0] - self.own_local_en[0], self.anchor[1] - self.own_local_en[1]))
        return [ue, un, vd]

    # --------------------------------------------------------------- main loop

    def control_loop(self):
        now = time.monotonic()
        self.tick += 1
        self.publish_offboard_control_mode()
        nb = self.neighbor(now)
        self.update_phase(now, nb)
        cmd = self.compute_command(now, nb)
        if self.phase != "LAND":
            # world (E, N, D) -> PX4 local NED (N, E, D)
            self.publish_setpoint([cmd[1], cmd[0], cmd[2]])
        self.broadcast_state()
        self.write_log(now, cmd)
        if self.tick % 10 == 0 and "rel" in self.dbg:
            want = math.hypot(self.des_e, self.des_n)
            self.get_logger().info(
                f"[{self.plan_label}] dist={math.hypot(*self.dbg['rel']):.2f} m "
                f"(D wants {want:.1f})  |rep|={math.hypot(*self.dbg['u_rep']):.2f} m/s  min={self.min_dist:.2f}")

    def broadcast_state(self):
        def f(x):
            return float(x) if x is not None and math.isfinite(x) else 0.0
        if DEPLOYMENT == "SITL" and self.own_world is not None:
            e, n = self.own_world
        else:
            e, n = 0.0, 0.0
        lat, lon = self.own_latlon if self.own_latlon is not None else (0.0, 0.0)
        valid = 1 if self.own_ready(time.monotonic()) else 0
        self.link.send(dict(id=self.drone_id, phase=self.phase, valid=valid, plan_idx=int(self.plan_idx),
                            label=self.plan_label, des_e=f(self.des_e), des_n=f(self.des_n),
                            g_e=f(self.g_e), g_n=f(self.g_n), m_e=f(self.m_e), m_n=f(self.m_n), mode=self.step_mode, ff=1 if self.ff else 0,
                            leader=-1 if self.leader_id is None else int(self.leader_id),
                            e=f(e), n=f(n), ve=f(self.own_ve), vn=f(self.own_vn), lat=f(lat), lon=f(lon)))

    def write_log(self, now, cmd):
        d = self.dbg
        t = f"{now - self.t_start:.2f}"
        want = math.hypot(self.des_e, self.des_n)
        if "rel" in d:
            rel, des, uf, ur = d["rel"], d["des"], d["u_form"], d["u_rep"]
            dist = math.hypot(*rel)
            err = math.hypot(rel[0] - des[0], rel[1] - des[1])
            td = "" if d["true_d"] is None else f"{d['true_d']:.3f}"
            chk = d["true_d"] if d["true_d"] is not None else dist
            row = [t, self.phase, self.plan_label, self.step_mode, f"{self.des_e:.2f}", f"{self.des_n:.2f}",
                   f"{self.g_e:.2f}", f"{self.g_n:.2f}", f"{self.m_e:.2f}", f"{self.m_n:.2f}", f"{d['anchor_err']:.3f}",
                   f"{want:.2f}", f"{dist:.3f}", td, f"{err:.3f}", int(chk < CRIT_RADIUS), f"{rel[0]:.3f}", f"{rel[1]:.3f}",
                   f"{uf[0]:.3f}", f"{uf[1]:.3f}", f"{ur[0]:.3f}", f"{ur[1]:.3f}",
                   f"{cmd[0]:.3f}", f"{cmd[1]:.3f}", f"{cmd[2]:.3f}",
                   f"{self.own_ve:.3f}", f"{self.own_vn:.3f}", f"{d['age']:.3f}", int(self.ff)]
        else:
            row = [t, self.phase, self.plan_label, self.step_mode, f"{self.des_e:.2f}", f"{self.des_n:.2f}",
                   f"{self.g_e:.2f}", f"{self.g_n:.2f}", f"{self.m_e:.2f}", f"{self.m_n:.2f}", "",
                   f"{want:.2f}", "", "", "", "", "", "", "", "", "", "",
                   f"{cmd[0]:.3f}", f"{cmd[1]:.3f}", f"{cmd[2]:.3f}",
                   f"{self.own_ve:.3f}", f"{self.own_vn:.3f}", "", int(self.ff)]
        self.csv.writerow(row)
        if self.tick % 10 == 0:
            self.csv_file.flush()

    # ------------------------------------------------------- publishing helpers

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, velocity_ned):
        nan3 = [float("nan")] * 3
        msg = TrajectorySetpoint()
        msg.position = list(nan3)
        msg.velocity = [float(v) for v in velocity_ned]
        msg.acceleration = list(nan3)
        msg.jerk = list(nan3)
        msg.yaw = float(self.hold_yaw)       # hold the starting heading: no rotation at takeoff
        msg.yawspeed = float("nan")
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def publish_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = self.mav_sys_id
        msg.target_component = 1
        msg.source_system = self.mav_sys_id
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def arm(self):
        self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard(self):
        self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def emergency_land_on_exit(self):
        """Ctrl+C while flying: don't leave PX4 waiting on a dead offboard stream."""
        self.report_min_dist()
        if self.phase in AIRBORNE_PHASES:
            for _ in range(5):
                self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                time.sleep(0.05)
        self.link.close()
        self.csv_file.close()


def configure(args):
    """Apply the command-line options to the module settings (before the node is created)."""
    global FORMATION_PLAN, PLAN_NAME, LEADER_ID, ALT_STAGGER, AVOID_SWIRL_DEG
    global SIM_SEP_ERROR, SIM_REL_NOISE_STD, SIM_PACKET_DROP
    global GROUP_FEEDFORWARD, KP, KV
    global PLANS
    ms, mt = getattr(args, "move_speed", None), getattr(args, "move_time", None)
    if ms is not None and not (0.0 < ms <= MOVE_SPEED_MAX):
        raise SystemExit(f"--move-speed must be between 0 and {MOVE_SPEED_MAX} m/s (the formation pull is capped at "
                         f"{FORMATION_MAX_SPEED} m/s and avoidance needs headroom).")
    if mt is not None and mt <= 0.0:
        raise SystemExit("--move-time must be positive.")
    PLANS = _build_plans(ms, mt)
    PLAN_NAME = args.plan
    FORMATION_PLAN = PLANS[PLAN_NAME]
    if args.leader is not None:
        LEADER_ID = None if args.leader == "none" else int(args.leader)
    if getattr(args, "ff", None) is not None:
        GROUP_FEEDFORWARD = (args.ff == "on")
    if getattr(args, "kp", None) is not None:
        KP = float(args.kp)
    if getattr(args, "kv", None) is not None:
        KV = float(args.kv)
    if not (0.0 < KP <= 1.5) or not (0.0 <= KV <= 2.0):
        raise SystemExit("--kp must be in (0, 1.5] and --kv in [0, 2] (the pure first-order law is --kp 1 --kv 0).")
    if not GROUP_FEEDFORWARD and LEADER_ID is None:
        raise SystemExit("--ff off needs a leader (--leader 0 or 1): with the law alone and no leader, nothing "
                         "would move the formation.")
    if args.stagger is not None:
        ALT_STAGGER = {0: 0.0, 1: float(args.stagger)}
    if args.swirl is not None:
        AVOID_SWIRL_DEG = float(args.swirl)
    stress = (args.sep_error, args.noise, args.drop)
    if DEPLOYMENT == "REAL":
        if any(v for v in stress):
            raise SystemExit("--sep-error / --noise / --drop are simulation-only options and are refused in REAL mode.")
    else:
        SIM_SEP_ERROR = float(args.sep_error or 0.0)
        SIM_REL_NOISE_STD = float(args.noise or 0.0)
        SIM_PACKET_DROP = min(max(float(args.drop or 0.0), 0.0), 1.0)
    if args.peer_ip:
        PEER_IP[PEER_ID[args.id]] = args.peer_ip


def run(drone_id):
    rclpy.init()
    node = TwoDroneFormationNode(drone_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.emergency_land_on_exit()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Two-drone formation CHANGE + collision avoidance test")
    parser.add_argument("--id", type=int, required=True, choices=[0, 1],
                        help="0 = LEFT/West drone, 1 = RIGHT/East drone")
    parser.add_argument("--plan", choices=sorted(PLANS), default=PLAN_NAME,
                        help="which formation change to run (only drone 0's choice is used)")
    parser.add_argument("--move-speed", type=float, default=None,
                        help=f"group speed in the MOVE steps of the mission plans, m/s (default {MOVE_SPEED}, max {MOVE_SPEED_MAX})")
    parser.add_argument("--move-time", type=float, default=None,
                        help=f"duration of each MOVE step in seconds (default {MOVE_SECONDS:.0f}); distance = speed x time")
    parser.add_argument("--leader", choices=["0", "1", "none"], default=None,
                        help="which drone holds position; 'none' = both drones move (only drone 0's choice is used)")
    parser.add_argument("--stagger", type=float, default=None,
                        help="extra height (m) for drone 1. Set on drone 1's terminal. REAL default 1.0")
    parser.add_argument("--swirl", type=float, default=None, help="avoidance sidestep angle in degrees (0 = pure paper)")
    parser.add_argument("--ff", choices=["on", "off"], default=None,
                        help="group-velocity feed-forward for the follower during MOVE steps (default on). "
                             "'off' = displacement law alone; needs a leader. Only drone 0's choice is used.")
    parser.add_argument("--kp", type=float, default=None, help=f"formation gain KP (default {KP}); the paper's first-order law is 1")
    parser.add_argument("--kv", type=float, default=None, help=f"relative-velocity damping KV (default {KV}); the paper's first-order law is 0")
    parser.add_argument("--peer-ip", default=None, help="IP address of the OTHER drone's Pi / of the ghost laptop")
    parser.add_argument("--sep-error", type=float, default=None, help="SITL only: other drone looks this many m farther")
    parser.add_argument("--noise", type=float, default=None, help="SITL only: relative-position noise std (m)")
    parser.add_argument("--drop", type=float, default=None, help="SITL only: probability of dropping a received packet")
    args, _ = parser.parse_known_args()
    configure(args)
    run(args.id)


if __name__ == "__main__":
    main()
