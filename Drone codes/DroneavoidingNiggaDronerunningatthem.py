#!/usr/bin/env python3
"""
RC-approach avoidance test (PX4 + ROS 2, Raspberry Pi on each drone)

THE TEST
    Drone A (the AVOIDER) takes off and hovers on its own (Offboard mode).
    Drone B is flown by a person with the RC transmitter and is brought
    towards A. When B comes inside D_SENSE of A, A moves away from B (the
    collision-avoidance potential of Dimarogonas & Kyriakopoulos 2006). When B
    leaves, A drifts back to its hover spot.

THREE ROLES, ONE FILE (all share the same UDP packet format)
    --role avoider   on drone A's Pi. Flies A: takeoff, hover, avoid, land.
    --role beacon    on drone B's Pi. Sends B's position/velocity to A. It only READS
                     telemetry and never sends a command, so the RC pilot keeps full control.
    --role intruder  NO drone needed (no ROS, can run on a laptop). A virtual drone
                     that walks towards A along a set path. Use it first: it tests A's
                     reaction safely before a real second drone is involved.

    Run order for the first real test: avoider + intruder (only ONE drone flies).
    Then avoider + beacon with a real RC-flown drone B.

USAGE
    drone A's Pi :  python3 rc_approach_avoidance_test.py --role avoider
    drone B's Pi :  python3 rc_approach_avoidance_test.py --role beacon
    anywhere     :  python3 rc_approach_avoidance_test.py --role intruder
    Press ENTER in the avoider terminal to arm + take off, ENTER again to land.

SAFETY (read before flying)
    - Props OFF first. RC transmitter in hand for BOTH drones (two people). This
      script is not a failsafe; the avoider cannot outrun a fast drone.
    - The RC pilot approaches SLOWLY (under 1 m/s), starts well away (10 m or more),
      and for the first tries flies about 1 m higher or lower than A. Avoidance only
      looks at horizontal distance, so the height difference is just a safety margin.
    - A can only retreat at MAX_SPEED_XY (1 m/s). In simulation a 2 m/s head-on approach
      got to 1.5 m even with the sidestep, so a fast drone cannot be avoided.
    - Land if A reports CORNERED (it will not retreat further than MAX_RETREAT from its spot).
    - A lands by itself if it stops hearing the other drone for LINK_LOST_LAND_SEC,
      after TEST_DURATION_SEC, or on Ctrl+C.
    - Yaw is held at A's starting heading (no rotation at takeoff).

LOG
    avoider_log_<time>.csv in the current folder (10 Hz): distance, push, commands.
"""

import argparse
import csv
import json
import math
import socket
import threading
import time

try:
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
    ROS_OK = True
except ImportError:           # the intruder role runs fine without ROS
    ROS_OK = False

    class Node:               # placeholder so the class definitions below still load
        pass

# ---------------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------------

DEPLOYMENT = "REAL"   # "REAL" (GPS, one Pi per drone) or "SITL" (everything on one PC)

# --- Topics (check with `ros2 topic list | grep fmu/out`) ---
LOCAL_POSITION_TOPIC = "vehicle_local_position_v1"
GLOBAL_POSITION_TOPIC = "vehicle_global_position"      # REAL only; may carry a version suffix
VEHICLE_STATUS_TOPIC = "vehicle_status_v1" if DEPLOYMENT == "REAL" else "vehicle_status_v4"
NAMESPACE = ""        # bare /fmu/... on each Pi. SITL: "" for the avoider, "/px4_1" for the beacon.
BEACON_NAMESPACE = "/px4_1" if DEPLOYMENT == "SITL" else ""

# --- The avoider drone's MAV_SYS_ID (`param show MAV_SYS_ID`) ---
AVOIDER_MAV_SYS_ID = 1

# --- UDP link: AVOIDER_IP = avoider Pi, OTHER_IP = the Pi (or laptop) running beacon/intruder ---
AVOIDER_IP = "CHANGE_ME" if DEPLOYMENT == "REAL" else "127.0.0.1"
OTHER_IP = "CHANGE_ME" if DEPLOYMENT == "REAL" else "127.0.0.1"
AVOIDER_PORT = 14700
OTHER_PORT = 14701
NEIGHBOR_TIMEOUT_SEC = 0.5
LINK_LOST_LAND_SEC = 5.0
MAX_EXTRAPOLATE_SEC = 0.5

# --- SITL only: where each vehicle spawns in the shared world (East, North), metres ---
SPAWN_WORLD_OFFSET = {"avoider": (0.0, 0.0), "beacon": (12.0, 0.0)}

# --- Mission ---
TAKEOFF_HEIGHT = 2.0
ARRIVAL_THRESHOLD = 0.3
MIN_START_SEPARATION = 6.0     # refuse takeoff if the other drone is closer than this
MAX_START_SEPARATION = 60.0
TEST_DURATION_SEC = 240.0      # time spent in HOVER before it lands by itself

# --- Hold-position gains ---
ANCHOR_KP = 0.5
ANCHOR_MAX_SPEED = 0.5         # must stay below REPULSION_SPEED_AT_CRITICAL
ALT_KP = 0.5
MAX_SPEED_Z = 0.6
MAX_SPEED_XY = 1.0
MAX_RETREAT = 8.0              # m: never retreat further than this from the hover spot

# --- Collision avoidance potential (Dimarogonas 2006) ---
D_SENSE = 5.0                  # avoidance is exactly zero beyond this distance
REPULSION_SPEED_AT_CRITICAL = 1.0
AVOID_SWIRL_DEG = 30.0         # 30 = also sidestep to the right (recommended: a head-on approach otherwise
                               # pushes A straight back to its retreat limit). 0 = exactly the paper.
assert ANCHOR_MAX_SPEED < REPULSION_SPEED_AT_CRITICAL, "avoidance must out-push the hold-position pull"
_C = D_SENSE ** 2 / 3.0                                   # paper: d^2 = 3c
REP_H = REPULSION_SPEED_AT_CRITICAL / (8.0 * _C ** 1.5)   # paper: a = 4*h*c^3

# --- Virtual intruder path, relative to A's hover spot (East, North) in metres ---
INTRUDER_PATH = [(12.0, 2.0), (-2.0, 2.0), (-2.0, 2.0), (12.0, 2.0)]   # in, pause, out
INTRUDER_SPEED = 0.6           # m/s
INTRUDER_PAUSE_SEC = 5.0       # pause at the closest point

TAKEOFF_TIMEOUT_SEC = 25.0
CONTROL_PERIOD_SEC = 0.1

R_EARTH = 6371000.0

# ---------------------------------------------------------------------------
# PURE MATH
# ---------------------------------------------------------------------------


def ll_to_en_diff(lat_a, lon_a, lat_b, lon_b):
    """(east, north) of A relative to B in metres (flat earth, fine at tens of metres)."""
    north = math.radians(lat_a - lat_b) * R_EARTH
    east = math.radians(lon_a - lon_b) * R_EARTH * math.cos(math.radians((lat_a + lat_b) / 2.0))
    return east, north


def en_to_ll(lat0, lon0, east, north):
    lat = lat0 + math.degrees(north / R_EARTH)
    lon = lon0 + math.degrees(east / (R_EARTH * math.cos(math.radians(lat0))))
    return lat, lon


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
    """-dV/dq_i = -2 rho (q_i - q_j): pushes drone i straight away from drone j."""
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


# ---------------------------------------------------------------------------
# UDP LINK
# ---------------------------------------------------------------------------

_REQUIRED_KEYS = ("id", "phase", "e", "n", "ve", "vn", "lat", "lon")


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
                if pkt["id"] == self.my_id:
                    continue
                for k in ("e", "n", "ve", "vn", "lat", "lon"):
                    if not isinstance(pkt[k], (int, float)) or not math.isfinite(pkt[k]):
                        raise ValueError(k)
            except (ValueError, UnicodeDecodeError):
                continue
            with self.lock:
                self.latest[pkt["id"]] = (time.monotonic(), pkt)

    def get(self, peer_id):
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


def _f(x):
    return float(x) if x is not None and math.isfinite(x) else 0.0


def relative_en(own_world, own_latlon, pkt, age):
    """q_own - q_other (East, North), the other drone predicted forward by v*age."""
    if DEPLOYMENT == "REAL":
        e, n = ll_to_en_diff(own_latlon[0], own_latlon[1], pkt["lat"], pkt["lon"])
    else:
        e, n = own_world[0] - pkt["e"], own_world[1] - pkt["n"]
    age = min(age, MAX_EXTRAPOLATE_SEC)
    return (e - pkt["ve"] * age, n - pkt["vn"] * age)


# ---------------------------------------------------------------------------
# ROLE 1: AVOIDER (flies drone A)
# ---------------------------------------------------------------------------


class AvoiderNode(Node):
    def __init__(self):
        super().__init__("rc_approach_avoider")
        if DEPLOYMENT == "REAL" and "CHANGE_ME" in (AVOIDER_IP, OTHER_IP):
            raise RuntimeError("Set AVOIDER_IP and OTHER_IP (hostname -I on each Pi) before running.")
        self.mav_sys_id = AVOIDER_MAV_SYS_ID
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        ns = NAMESPACE
        self.offboard_pub = self.create_publisher(OffboardControlMode, f"{ns}/fmu/in/offboard_control_mode", qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, f"{ns}/fmu/in/trajectory_setpoint", qos)
        self.command_pub = self.create_publisher(VehicleCommand, f"{ns}/fmu/in/vehicle_command", qos)
        self.create_subscription(VehicleLocalPosition, f"{ns}/fmu/out/{LOCAL_POSITION_TOPIC}", self.local_cb, qos)
        self.create_subscription(VehicleStatus, f"{ns}/fmu/out/{VEHICLE_STATUS_TOPIC}", self.status_cb, qos)
        if DEPLOYMENT == "REAL":
            self.create_subscription(VehicleGlobalPosition, f"{ns}/fmu/out/{GLOBAL_POSITION_TOPIC}",
                                     self.global_cb, qos)
        self.link = PeerLink(0, AVOIDER_PORT, [(OTHER_IP, OTHER_PORT)])

        self.local_valid, self.last_local_time = False, -1e9
        self.own_local_en = (0.0, 0.0)
        self.own_world, self.own_latlon = None, None
        self.own_ve = self.own_vn = 0.0
        self.own_z = 0.0
        self.heading = None
        self.vehicle_status = VehicleStatus()

        self.phase, self.phase_t0 = "WAIT_DATA", time.monotonic()
        self.go = self.land_requested = False
        self.anchor = (0.0, 0.0)
        self.target_z = 0.0
        self.hold_yaw = 0.0
        self.ticks = 0
        self.last_fresh = time.monotonic()
        self.hover_t0 = 0.0
        self.land_t0 = self.last_resend = 0.0
        self.cornered = False
        self._last_log = {}
        self.dbg = {}
        self.tick = 0
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_file = open(f"avoider_log_{stamp}.csv", "w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(["t_s", "phase", "dist_h", "rel_e", "rel_n", "u_anchor_e", "u_anchor_n",
                           "u_rep_e", "u_rep_n", "cmd_e", "cmd_n", "cmd_d", "from_anchor_m", "nb_age"])
        self.t_start = time.monotonic()
        self.timer = self.create_timer(CONTROL_PERIOD_SEC, self.control_loop)
        self.get_logger().info(
            f"Avoider started ({DEPLOYMENT}), mav_sys_id={self.mav_sys_id}, D_SENSE={D_SENSE} m, "
            f"critical radius={D_SENSE / math.sqrt(3):.2f} m, max retreat={MAX_RETREAT} m")

    # ---- helpers ----
    def log_every(self, key, period, level, msg):
        now = time.monotonic()
        if now - self._last_log.get(key, -1e9) >= period:
            self._last_log[key] = now
            getattr(self.get_logger(), level)(msg)

    def set_phase(self, p):
        self.get_logger().info(f"Phase: {self.phase} -> {p}")
        self.phase, self.phase_t0 = p, time.monotonic()

    # ---- callbacks ----
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
            sx, sy = SPAWN_WORLD_OFFSET["avoider"]
            self.own_world = (sx + msg.y, sy + msg.x)

    def global_cb(self, msg):
        self.own_latlon = (msg.lat, msg.lon) if msg.lat_lon_valid else None

    def status_cb(self, msg):
        self.vehicle_status = msg

    def own_ready(self, now):
        if not self.local_valid or self.heading is None or now - self.last_local_time > 1.0:
            return False
        return (self.own_latlon is not None) if DEPLOYMENT == "REAL" else (self.own_world is not None)

    def neighbor(self):
        item = self.link.get(1)
        return None if item is None or item[0] > NEIGHBOR_TIMEOUT_SEC else item

    # ---- ENTER prompts ----
    def prompt_takeoff(self):
        def worker():
            try:
                input("\n>>> Other drone / intruder running and far away? Press ENTER to ARM + TAKE OFF: ")
                self.go = True
            except (EOFError, OSError):
                self.get_logger().error("No terminal attached: cannot ask for ENTER, so it will not take off.")
        threading.Thread(target=worker, daemon=True).start()

    def prompt_land(self):
        def worker():
            try:
                input("\n>>> Press ENTER at any time to LAND: ")
                self.land_requested = True
            except (EOFError, OSError):
                pass
        threading.Thread(target=worker, daemon=True).start()

    # ---- phase machine ----
    def start_landing(self, reason):
        self.get_logger().warn(f"LANDING: {reason}")
        self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.land_t0 = self.last_resend = time.monotonic()
        self.set_phase("LAND")

    def update_phase(self, now, nb):
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
                self.try_takeoff(now, nb)
            return
        if ph == "LAND":
            if now - self.land_t0 < 10.0 and now - self.last_resend >= 1.0:
                self.last_resend = now
                self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            return
        # airborne (TAKEOFF / HOVER)
        if nb is not None:
            self.last_fresh = now
        elif now - self.last_fresh > LINK_LOST_LAND_SEC:
            self.start_landing(f"no data from the other drone for {LINK_LOST_LAND_SEC:.0f} s")
            return
        else:
            self.log_every("stale", 1.0, "warn", "Other drone's data is stale: holding position")
        if self.land_requested:
            self.start_landing("ENTER pressed")
            return
        if self.ticks >= 30 and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.log_every("ob", 5.0, "warn", "PX4 is not in Offboard mode (RC takeover, failsafe, or status "
                           "topic not arriving?)")
        if ph == "TAKEOFF":
            self.ticks += 1
            if self.ticks == 10:
                self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info("Offboard + arm commands sent")
            if abs(self.own_z - self.target_z) < ARRIVAL_THRESHOLD and self.ticks > 20:
                self.hover_t0 = now
                self.set_phase("HOVER")
            elif now - self.phase_t0 > TAKEOFF_TIMEOUT_SEC:
                self.start_landing("takeoff timed out (did it arm? check QGC messages)")
        elif ph == "HOVER":
            if now - self.hover_t0 > TEST_DURATION_SEC:
                self.start_landing("test time finished")

    def try_takeoff(self, now, nb):
        if not self.own_ready(now):
            self.get_logger().error("Refusing takeoff: own position/heading not valid.")
        elif nb is None:
            self.get_logger().error("Refusing takeoff: no fresh data from the other drone/intruder. "
                                    "Check Wi-Fi, IPs, and that the beacon/intruder is running.")
        else:
            age, pkt = nb
            rel = relative_en(self.own_world, self.own_latlon, pkt, age)
            sep = math.hypot(*rel)
            if not (MIN_START_SEPARATION <= sep <= MAX_START_SEPARATION):
                self.get_logger().error(f"Refusing takeoff: the other drone is {sep:.1f} m away, outside "
                                        f"[{MIN_START_SEPARATION}, {MAX_START_SEPARATION}] m.")
            else:
                self.anchor = self.own_local_en
                self.target_z = self.own_z - TAKEOFF_HEIGHT
                self.hold_yaw = self.heading
                self.ticks, self.last_fresh = 0, now
                self.get_logger().info(f"Takeoff OK: other drone {sep:.1f} m away, heading "
                                       f"{math.degrees(self.hold_yaw):.0f} deg")
                self.set_phase("TAKEOFF")
                self.prompt_land()
                return
        self.prompt_takeoff()

    # ---- control law ----
    def compute_command(self, now, nb):
        self.dbg = {}
        if self.phase in ("WAIT_DATA", "WAIT_GO", "LAND"):
            return [0.0, 0.0, 0.0]
        if not self.own_ready(now):
            self.log_every("inv", 1.0, "warn", "Own position invalid: commanding zero velocity")
            return [0.0, 0.0, 0.0]
        vd = clamp(ALT_KP * (self.target_z - self.own_z), MAX_SPEED_Z)
        ae, an = self.anchor[0] - self.own_local_en[0], self.anchor[1] - self.own_local_en[1]
        u_anchor = clamp_vec((ANCHOR_KP * ae, ANCHOR_KP * an), ANCHOR_MAX_SPEED)
        if self.phase == "TAKEOFF":
            return [u_anchor[0], u_anchor[1], vd]
        u_rep, rel, age = (0.0, 0.0), None, None
        if nb is not None:
            age, pkt = nb
            rel = relative_en(self.own_world, self.own_latlon, pkt, age)
            u_rep = repulsion_velocity(rel, D_SENSE, REP_H, 1.0, AVOID_SWIRL_DEG)
        u = clamp_vec((u_anchor[0] + u_rep[0], u_anchor[1] + u_rep[1]), MAX_SPEED_XY)
        # leash: never retreat further than MAX_RETREAT from the hover spot
        da = math.hypot(ae, an)
        self.cornered = False
        if da >= MAX_RETREAT:
            out = (-ae / da, -an / da)               # unit vector pointing away from the anchor
            along = u[0] * out[0] + u[1] * out[1]
            if along > 0.0:
                u = (u[0] - along * out[0], u[1] - along * out[1])
                self.cornered = True
                self.log_every("corner", 2.0, "warn", "CORNERED: at the retreat limit. Pilot back off, or land.")
        self.dbg = dict(rel=rel, u_anchor=u_anchor, u_rep=u_rep, da=da, age=age)
        return [u[0], u[1], vd]

    # ---- main loop ----
    def control_loop(self):
        now = time.monotonic()
        self.tick += 1
        self.publish_offboard_control_mode()
        nb = self.neighbor()
        self.update_phase(now, nb)
        cmd = self.compute_command(now, nb)
        if self.phase != "LAND":
            self.publish_setpoint([cmd[1], cmd[0], cmd[2]])          # world (E,N,D) -> PX4 NED (N,E,D)
        self.broadcast_state()
        self.write_log(now, cmd)
        if self.tick % 10 == 0 and self.dbg.get("rel") is not None:
            self.get_logger().info(
                f"[{self.phase}] other drone {math.hypot(*self.dbg['rel']):.1f} m away, push "
                f"{math.hypot(*self.dbg['u_rep']):.2f} m/s, {self.dbg['da']:.1f} m from hover spot")

    def broadcast_state(self):
        e, n = self.own_world if (DEPLOYMENT == "SITL" and self.own_world) else (0.0, 0.0)
        lat, lon = self.own_latlon if self.own_latlon else (0.0, 0.0)
        self.link.send(dict(id=0, role="avoider", phase=self.phase, e=_f(e), n=_f(n), ve=_f(self.own_ve),
                            vn=_f(self.own_vn), lat=_f(lat), lon=_f(lon)))

    def write_log(self, now, cmd):
        d = self.dbg
        t = f"{now - self.t_start:.2f}"
        if d.get("rel") is not None:
            r, ua, ur = d["rel"], d["u_anchor"], d["u_rep"]
            self.csv.writerow([t, self.phase, f"{math.hypot(*r):.3f}", f"{r[0]:.3f}", f"{r[1]:.3f}",
                               f"{ua[0]:.3f}", f"{ua[1]:.3f}", f"{ur[0]:.3f}", f"{ur[1]:.3f}",
                               f"{cmd[0]:.3f}", f"{cmd[1]:.3f}", f"{cmd[2]:.3f}", f"{d['da']:.2f}", f"{d['age']:.3f}"])
        else:
            self.csv.writerow([t, self.phase, "", "", "", "", "", "", "", f"{cmd[0]:.3f}", f"{cmd[1]:.3f}",
                               f"{cmd[2]:.3f}", "", ""])
        if self.tick % 10 == 0:
            self.csv_file.flush()

    # ---- publishing ----
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
        if self.phase in ("TAKEOFF", "HOVER"):
            for _ in range(5):
                self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                time.sleep(0.05)
        self.link.close()
        self.csv_file.close()


# ---------------------------------------------------------------------------
# ROLE 2: BEACON (on the RC-flown drone: read-only, never commands PX4)
# ---------------------------------------------------------------------------


class BeaconNode(Node):
    def __init__(self):
        super().__init__("rc_approach_beacon")
        if DEPLOYMENT == "REAL" and "CHANGE_ME" in (AVOIDER_IP, OTHER_IP):
            raise RuntimeError("Set AVOIDER_IP and OTHER_IP before running.")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        ns = BEACON_NAMESPACE
        self.create_subscription(VehicleLocalPosition, f"{ns}/fmu/out/{LOCAL_POSITION_TOPIC}", self.local_cb, qos)
        if DEPLOYMENT == "REAL":
            self.create_subscription(VehicleGlobalPosition, f"{ns}/fmu/out/{GLOBAL_POSITION_TOPIC}",
                                     self.global_cb, qos)
        self.link = PeerLink(1, OTHER_PORT, [(AVOIDER_IP, AVOIDER_PORT)])
        self.valid, self.own_world, self.own_latlon = False, None, None
        self.ve = self.vn = 0.0
        self.tick = 0
        self.timer = self.create_timer(CONTROL_PERIOD_SEC, self.loop)
        self.get_logger().info("Beacon started: only READS telemetry and sends it to the avoider. "
                               "It never sends a command to PX4; you keep full RC control.")

    def local_cb(self, msg):
        self.valid = bool(msg.xy_valid and msg.z_valid)
        if self.valid:
            self.ve, self.vn = msg.vy, msg.vx
            if DEPLOYMENT == "SITL":
                sx, sy = SPAWN_WORLD_OFFSET["beacon"]
                self.own_world = (sx + msg.y, sy + msg.x)

    def global_cb(self, msg):
        self.own_latlon = (msg.lat, msg.lon) if msg.lat_lon_valid else None

    def loop(self):
        self.tick += 1
        ready = self.valid and (self.own_latlon is not None if DEPLOYMENT == "REAL" else self.own_world is not None)
        if not ready:
            self.log_every_n(30, "Waiting for a valid position (GPS lock)...")
            return
        e, n = self.own_world if DEPLOYMENT == "SITL" else (0.0, 0.0)
        lat, lon = self.own_latlon if self.own_latlon else (0.0, 0.0)
        self.link.send(dict(id=1, role="beacon", phase="BEACON", e=_f(e), n=_f(n), ve=_f(self.ve),
                            vn=_f(self.vn), lat=_f(lat), lon=_f(lon)))
        item = self.link.get(0)
        if item and item[0] <= NEIGHBOR_TIMEOUT_SEC * 2 and self.tick % 10 == 0:
            rel = relative_en(self.own_world, self.own_latlon, item[1], item[0])
            d = math.hypot(*rel)
            tag = "  <-- INSIDE avoidance zone" if d < D_SENSE else ""
            self.get_logger().info(f"Distance to the avoider: {d:.1f} m{tag}")

    def log_every_n(self, n, msg):
        if self.tick % n == 1:
            self.get_logger().info(msg)

    def shutdown(self):
        self.link.close()


# ---------------------------------------------------------------------------
# ROLE 3: VIRTUAL INTRUDER (no drone, no ROS)
# ---------------------------------------------------------------------------


class VirtualIntruder:
    """Walks INTRUDER_PATH relative to the avoider's hover spot and broadcasts itself as drone B."""

    def __init__(self):
        if DEPLOYMENT == "REAL" and "CHANGE_ME" in (AVOIDER_IP, OTHER_IP):
            raise RuntimeError("Set AVOIDER_IP and OTHER_IP before running.")
        self.link = PeerLink(1, OTHER_PORT, [(AVOIDER_IP, AVOIDER_PORT)])
        self.ref = None            # avoider's hover spot: (lat, lon) in REAL, (e, n) in SITL
        self.locked = False        # True once the avoider is hovering and the hover spot is fixed
        self.t0 = None
        self.pos = None            # current (East, North) offset from ref
        self.vel = (0.0, 0.0)
        self.n = 0

    def _segments(self):
        segs, t = [], 0.0
        pts = INTRUDER_PATH
        for a, b in zip(pts[:-1], pts[1:]):
            dist = math.hypot(b[0] - a[0], b[1] - a[1])
            if dist < 1e-6:
                segs.append((t, t + INTRUDER_PAUSE_SEC, a, a))
                t += INTRUDER_PAUSE_SEC
            else:
                dur = dist / INTRUDER_SPEED
                segs.append((t, t + dur, a, b))
                t += dur
        return segs

    def path_at(self, t):
        segs = self._segments()
        for t0, t1, a, b in segs:
            if t0 <= t < t1:
                f = (t - t0) / (t1 - t0)
                v = ((b[0] - a[0]) / (t1 - t0), (b[1] - a[1]) / (t1 - t0))
                return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])), v
        return segs[-1][3], (0.0, 0.0)

    def step(self, now):
        item = self.link.get(0)
        if item is None or item[0] > 1.0:
            self.locked, self.t0 = False, None       # avoider not heard: stop sending
            return
        pkt = item[1]
        if not self.locked:
            # park at the start of the path, tracking the avoider's spot, so the avoider sees us before takeoff
            self.ref = (pkt["lat"], pkt["lon"]) if DEPLOYMENT == "REAL" else (pkt["e"], pkt["n"])
            if pkt["phase"] == "HOVER":              # lock the hover spot and start walking
                self.locked = True
                self.t0 = now + 3.0
                print("Avoider is hovering: starting the approach in 3 s")
        t = max(0.0, now - self.t0) if self.locked else 0.0
        self.pos, self.vel = self.path_at(t)
        oe, on = self.pos
        if DEPLOYMENT == "REAL":
            lat, lon = en_to_ll(self.ref[0], self.ref[1], oe, on)
            e = n = 0.0
        else:
            lat = lon = 0.0
            e, n = self.ref[0] + oe, self.ref[1] + on
        self.link.send(dict(id=1, role="intruder", phase="INTRUDER", e=_f(e), n=_f(n), ve=_f(self.vel[0]),
                            vn=_f(self.vel[1]), lat=_f(lat), lon=_f(lon)))
        self.n += 1
        if self.n % 10 == 0:
            if DEPLOYMENT == "REAL":
                re_, rn_ = ll_to_en_diff(pkt["lat"], pkt["lon"], lat, lon)
            else:
                re_, rn_ = pkt["e"] - e, pkt["n"] - n
            state = "walking" if self.locked and now >= self.t0 else "waiting"
            print(f"Intruder {state} at ({oe:5.1f}, {on:4.1f}) m from the hover spot; distance to the avoider "
                  f"{math.hypot(re_, rn_):.1f} m")

    def run(self):
        print("Virtual intruder running. It waits until the avoider is hovering. Ctrl+C to stop.")
        try:
            while True:
                self.step(time.monotonic())
                time.sleep(CONTROL_PERIOD_SEC)
        except KeyboardInterrupt:
            pass
        finally:
            self.link.close()


# ---------------------------------------------------------------------------


def run_ros(node_cls):
    if not ROS_OK:
        raise SystemExit("rclpy / px4_msgs not found: source your ROS 2 workspace first.")
    rclpy.init()
    node = node_cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description="RC-approach avoidance test")
    p.add_argument("--role", required=True, choices=["avoider", "beacon", "intruder"])
    args, _ = p.parse_known_args()
    if args.role == "avoider":
        run_ros(AvoiderNode)
    elif args.role == "beacon":
        run_ros(BeaconNode)
    else:
        VirtualIntruder().run()


if __name__ == "__main__":
    main()
