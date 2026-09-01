"""
AutoNav Web Dashboard — backend bridge (job protocol, matches firmware
with the head-on/sidestep collision fix).

New in this rewrite:
  - Live collision watchdog: if two robots' most recent heartbeats both
    place them on the same node within a short freshness window, this is
    flagged immediately and loudly in the log — a real-time sanity check
    against the exact class of bug the simulation testing just caught and
    fixed. This should never fire; if it does, something regressed.
  - Distinguishes "idle yield" from "sidestep" in the log: both use the
    same underlying EVT_YIELDED event, but the bridge now checks the
    robot's last known state right before the event to tell whether it
    was a parked robot proactively clearing a path (idle yield) or a
    moving robot escaping an otherwise-unresolvable head-on (sidestep).

Install once:
    pip install websockets --break-system-packages

Run:
    python autonav_bridge.py

Then open autonav_dashboard.html in your browser.
"""

import asyncio
import json
import random
import struct
import time

import websockets

UDP_PORT = 4210
MAX_ROUTE_LEN = 20
MAX_EDGES = 60
BROADCAST_IP = "255.255.255.255"
WS_PORT = 8765
COLLISION_FRESHNESS_S = 2.0   # both heartbeats must be within this window to count

MSG_HEARTBEAT = 1
MSG_ROUTE = 2
MSG_EVENT = 3
MSG_GRAPH_CONFIG = 4
MSG_MODE_CMD = 5
MSG_JOB_ANNOUNCE = 6
MSG_JOB_CLAIM = 7
MSG_JOB_COMPLETE = 8
MSG_JOB_RESULT = 9

IDLE, PLANNING, MOVING, WAITING = 0, 1, 2, 3
EVT_WAITING = 1
EVT_RESUMED = 2
EVT_REROUTED = 3
EVT_YIELDED = 4
EVT_JOB_CLAIMED = 5
EVT_ARRIVED_PICKUP = 6
EVT_ARRIVED_DROPOFF = 7

HEARTBEAT_FORMAT = "<BBBBf"
ROUTE_FORMAT = f"<BB{MAX_ROUTE_LEN}BB"
EVENT_FORMAT = "<BBBB"
GRAPH_CONFIG_FORMAT = f"<BB{MAX_EDGES*2}B"
MODE_CMD_FORMAT = "<BBB"
JOB_ANNOUNCE_FORMAT = "<BBBBBB"  # type, job_id_hi, job_id_lo, pickup, dropoff, target_robot
JOB_CLAIM_FORMAT = "<BBBB"
JOB_COMPLETE_FORMAT = "<BBBB"
JOB_RESULT_FORMAT = "<BBBBBBB"

HEARTBEAT_SIZE = struct.calcsize(HEARTBEAT_FORMAT)
ROUTE_SIZE = struct.calcsize(ROUTE_FORMAT)
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
JOB_RESULT_SIZE = struct.calcsize(JOB_RESULT_FORMAT)

EDGES = [
    (1, 2), (1, 10), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 8), (3, 8), (8, 9),
    (9, 10), (10, 11), (11, 12), (12, 13), (13, 14), (8, 13), (14, 15), (15, 16),
    (16, 17), (17, 18), (18, 13), (18, 19), (19, 20), (11, 20), (6, 15),
]
VALID_NODES = sorted(set(n for e in EDGES for n in e))

robots = {}   # robot_id -> {"ip", "last_seen", "current_node", "state", "prev_state"}
jobs = {}
clients = set()
transport = None
next_job_id = 1


def log_text(text, level="info"):
    broadcast({"type": "log", "text": text, "ts": time.strftime("%H:%M:%S"), "level": level})


def broadcast(msg):
    if not clients:
        return
    data = json.dumps(msg)
    for ws in list(clients):
        asyncio.create_task(_safe_send(ws, data))


async def _safe_send(ws, data):
    try:
        await ws.send(data)
    except Exception:
        clients.discard(ws)


def check_collision_watchdog(moved_robot_id, node):
    """Flag it immediately if another robot's fresh heartbeat also claims
    this exact node right now. Should never fire — this is the live
    real-world counterpart to the simulation's collision check."""
    now = time.time()
    for rid, r in robots.items():
        if rid == moved_robot_id:
            continue
        if r.get("current_node") == node and (now - r.get("last_seen", 0)) <= COLLISION_FRESHNESS_S:
            log_text(f"*** COLLISION WATCHDOG *** Robot {moved_robot_id} and Robot {rid} "
                      f"both report node {node} within {COLLISION_FRESHNESS_S}s of each other!",
                      level="collision")
            broadcast({"type": "collision_warning", "robots": [moved_robot_id, rid], "node": node})


def handle_event(robot_id, event_type, node):
    r = robots.setdefault(robot_id, {})
    prev_state = r.get("state", IDLE)

    if event_type == EVT_YIELDED:
        if prev_state == IDLE:
            text = f"Robot {robot_id} yields (idle) -> stepping aside to node {node}"
        else:
            text = f"Robot {robot_id} sidesteps to node {node} — clearing an unresolvable head-on"
    else:
        names = {
            EVT_WAITING: f"Robot {robot_id} waiting before node {node}",
            EVT_RESUMED: f"Robot {robot_id} resumed moving",
            EVT_REROUTED: f"Robot {robot_id} rerouted around node {node}",
            EVT_JOB_CLAIMED: f"Robot {robot_id} claimed job, heading to pickup {node}",
            EVT_ARRIVED_PICKUP: f"Robot {robot_id} arrived at pickup {node}, loading",
            EVT_ARRIVED_DROPOFF: f"Robot {robot_id} delivered at node {node}",
        }
        text = names.get(event_type, f"Robot {robot_id} event {event_type}")

    log_text(text)
    broadcast({"type": "event", "robot_id": robot_id, "event_type": event_type, "node": node})


class UDPProtocol(asyncio.DatagramProtocol):
    def connection_made(self, tr):
        global transport
        transport = tr

    def datagram_received(self, data, addr):
        if len(data) < 1:
            return
        msg_type = data[0]

        if msg_type == MSG_HEARTBEAT and len(data) == HEARTBEAT_SIZE:
            _, robot_id, current_node, state, eta = struct.unpack(HEARTBEAT_FORMAT, data)
            r = robots.setdefault(robot_id, {})
            r["ip"] = addr[0]
            r["prev_state"] = r.get("state", state)
            r["current_node"] = current_node
            r["state"] = state
            r["eta"] = eta
            r["last_seen"] = time.time()
            check_collision_watchdog(robot_id, current_node)
            broadcast({"type": "heartbeat", "robot_id": robot_id, "current_node": current_node,
                       "state": state, "eta": eta})

        elif msg_type == MSG_ROUTE and len(data) == ROUTE_SIZE:
            unpacked = struct.unpack(ROUTE_FORMAT, data)
            robot_id = unpacked[1]
            route = unpacked[2:2 + MAX_ROUTE_LEN]
            route_len = unpacked[2 + MAX_ROUTE_LEN]
            broadcast({"type": "route", "robot_id": robot_id, "route": list(route[:route_len])})

        elif msg_type == MSG_EVENT and len(data) == EVENT_SIZE:
            _, robot_id, event_type, node = struct.unpack(EVENT_FORMAT, data)
            handle_event(robot_id, event_type, node)

        elif msg_type == MSG_JOB_ANNOUNCE and len(data) == struct.calcsize(JOB_ANNOUNCE_FORMAT):
            _, hi, lo, pickup, dropoff, target_robot = struct.unpack(JOB_ANNOUNCE_FORMAT, data)
            jid = (hi << 8) | lo
            if jid not in jobs:
                jobs[jid] = {"pickup": pickup, "dropoff": dropoff, "status": 0, "claimed_by": 0,
                             "target_robot": target_robot}
                broadcast({"type": "job_announce", "job_id": jid, "pickup": pickup, "dropoff": dropoff,
                           "target_robot": target_robot})
                reserved = f" (reserved for robot {target_robot})" if target_robot else ""
                log_text(f"Job {jid} announced: {pickup} -> {dropoff}{reserved}")

        elif msg_type == MSG_JOB_CLAIM and len(data) == struct.calcsize(JOB_CLAIM_FORMAT):
            _, hi, lo, robot_id = struct.unpack(JOB_CLAIM_FORMAT, data)
            jid = (hi << 8) | lo
            if jid in jobs:
                jobs[jid]["status"] = 1
                jobs[jid]["claimed_by"] = robot_id
                broadcast({"type": "job_claim", "job_id": jid, "robot_id": robot_id})

        elif msg_type == MSG_JOB_COMPLETE and len(data) == struct.calcsize(JOB_COMPLETE_FORMAT):
            _, hi, lo, robot_id = struct.unpack(JOB_COMPLETE_FORMAT, data)
            jid = (hi << 8) | lo
            if jid in jobs:
                jobs[jid]["status"] = 2
                broadcast({"type": "job_complete", "job_id": jid, "robot_id": robot_id})

        elif msg_type == MSG_JOB_RESULT and len(data) == JOB_RESULT_SIZE:
            _, hi, lo, robot_id, mode, dur_hi, dur_lo = struct.unpack(JOB_RESULT_FORMAT, data)
            jid = (hi << 8) | lo
            duration_ms = ((dur_hi << 8) | dur_lo) * 100
            broadcast({"type": "job_result", "job_id": jid, "robot_id": robot_id,
                       "mode": mode, "duration_ms": duration_ms})
            mode_name = "SMART" if mode else "NAIVE"
            log_text(f"Job {jid} delivered by robot {robot_id} in {duration_ms/1000:.1f}s [{mode_name}]")


def send_packet(packet, ip=None):
    if transport is None:
        return
    transport.sendto(packet, (ip or BROADCAST_IP, UDP_PORT))


def send_to_all(packet):
    """Broadcast AND unicast directly to every robot IP we've already heard
    from. Broadcasting alone can silently fail to leave a laptop with
    multiple network adapters (WiFi + Ethernet/VPN) — it can go out the
    wrong interface. Direct unicast to a known IP doesn't have that
    problem, so this sends both: broadcast as the cheap first attempt,
    direct unicast to each known robot as real insurance."""
    send_packet(packet)  # broadcast attempt
    for rid, r in robots.items():
        ip = r.get("ip")
        if ip:
            send_packet(packet, ip)


def send_graph_config():
    flat = []
    for a, b in EDGES[:MAX_EDGES]:
        flat.extend([a, b])
    flat += [0] * (MAX_EDGES * 2 - len(flat))
    packet = struct.pack(GRAPH_CONFIG_FORMAT, MSG_GRAPH_CONFIG, len(EDGES), *flat)
    send_to_all(packet)
    log_text(f"Pushed graph config ({len(EDGES)} edges) to all robots")


def send_mode(smart):
    packet = struct.pack(MODE_CMD_FORMAT, MSG_MODE_CMD, 0, 1 if smart else 0)
    send_to_all(packet)
    log_text(f"Mode set to {'SMART' if smart else 'NAIVE'} for all robots")


def create_job(pickup, dropoff, target_robot=0):
    global next_job_id
    if pickup == dropoff or pickup not in VALID_NODES or dropoff not in VALID_NODES:
        log_text(f"Rejected malformed job: {pickup} -> {dropoff}", level="warn")
        return
    jid = next_job_id
    next_job_id += 1
    if next_job_id > 65000:
        next_job_id = 1
    jobs[jid] = {"pickup": pickup, "dropoff": dropoff, "status": 0, "claimed_by": 0, "target_robot": target_robot}
    packet = struct.pack(JOB_ANNOUNCE_FORMAT, MSG_JOB_ANNOUNCE, (jid >> 8) & 0xFF, jid & 0xFF,
                          pickup, dropoff, target_robot)
    send_to_all(packet)
    broadcast({"type": "job_announce", "job_id": jid, "pickup": pickup, "dropoff": dropoff,
               "target_robot": target_robot})
    reserved = f" (assigned to robot {target_robot})" if target_robot else ""
    log_text(f"Created job {jid}: {pickup} -> {dropoff}{reserved}")


def random_job():
    pickup, dropoff = random.sample(VALID_NODES, 2)
    create_job(pickup, dropoff)


async def ws_handler(websocket):
    clients.add(websocket)
    log_text("Dashboard connected")
    for jid, j in jobs.items():
        await websocket.send(json.dumps({"type": "job_announce", "job_id": jid,
                                          "pickup": j["pickup"], "dropoff": j["dropoff"],
                                          "target_robot": j.get("target_robot", 0)}))
        if j["status"] >= 1:
            await websocket.send(json.dumps({"type": "job_claim", "job_id": jid, "robot_id": j["claimed_by"]}))
        if j["status"] == 2:
            await websocket.send(json.dumps({"type": "job_complete", "job_id": jid, "robot_id": j["claimed_by"]}))
    try:
        async for message in websocket:
            try:
                cmd = json.loads(message)
            except Exception:
                continue
            action = cmd.get("cmd")
            if action == "create_job":
                create_job(int(cmd["pickup"]), int(cmd["dropoff"]), int(cmd.get("target_robot", 0)))
            elif action == "random_job":
                random_job()
            elif action == "push_graph":
                send_graph_config()
            elif action == "set_mode":
                send_mode(bool(cmd["smart"]))
    finally:
        clients.discard(websocket)


async def main():
    loop = asyncio.get_running_loop()
    await loop.create_datagram_endpoint(
        UDPProtocol, local_addr=("0.0.0.0", UDP_PORT), allow_broadcast=True
    )
    send_graph_config()
    async with websockets.serve(ws_handler, "localhost", WS_PORT):
        print(f"Bridge running. UDP on port {UDP_PORT}, WebSocket on ws://localhost:{WS_PORT}")
        print("Now open autonav_dashboard.html in your browser.")
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
