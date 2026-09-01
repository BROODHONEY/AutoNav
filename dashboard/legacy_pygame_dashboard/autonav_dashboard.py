"""
AutoNav Dashboard v3 — Tier 1: comparison mode, stress test, auto-queue.

New in this version:
  - Smart/naive mode toggle ('m'): broadcasts to all robots, switching
    between your smart coordination logic and a plain stop-and-wait
    baseline. Run the same task set in both modes and compare.
  - Live comparison stats: average task completion time for smart vs
    naive runs, computed from real TaskResult messages — this is the
    actual number your PS's success criteria asks for.
  - Stress test ('t'): instantly assigns every known robot the CURRENT
    node of another robot, guaranteeing a head-on conflict on demand,
    instead of hoping one happens naturally during a live demo.
  - Auto-queue toggle ('q'): robots automatically pick a new random task
    the moment they arrive, keeping the fleet continuously busy.

Run this on a laptop connected to the SAME WiFi network as the robots.

Install once:
    pip install pygame --break-system-packages

Run:
    python autonav_dashboard.py

Keys:
    g       resend the graph config to all robots
    m       toggle smart / naive mode for all robots
    q       toggle auto-queue for all robots
    t       stress test: force a guaranteed conflict between known robots
    c       clear comparison stats (start a fresh timing run)
    Esc     deselect the currently selected robot
"""

import socket
import struct
import time
import pygame

UDP_PORT = 4210
MAX_ROUTE_LEN = 20
MAX_EDGES = 60
BROADCAST_IP = "255.255.255.255"

MSG_HEARTBEAT = 1
MSG_ROUTE = 2
MSG_EVENT = 3
MSG_TASK_CMD = 4
MSG_GRAPH_CONFIG = 5
MSG_MODE_CMD = 6
MSG_AUTOQ_CMD = 7
MSG_TASK_RESULT = 8

EVT_TASK_START = 1
EVT_WAITING = 2
EVT_RESUMED = 3
EVT_REROUTED = 4
EVT_YIELDED = 5
EVT_ARRIVED = 6

HEARTBEAT_FORMAT = "<BBBBf"
ROUTE_FORMAT = f"<BB{MAX_ROUTE_LEN}BB"
EVENT_FORMAT = "<BBBB"
TASK_CMD_FORMAT = "<BBB"
GRAPH_CONFIG_FORMAT = f"<BB{MAX_EDGES*2}B"
MODE_CMD_FORMAT = "<BBB"
AUTOQ_CMD_FORMAT = "<BBB"
TASK_RESULT_FORMAT = "<BBBBI"   # type, robot_id, target_node, mode, duration_ms

HEARTBEAT_SIZE = struct.calcsize(HEARTBEAT_FORMAT)
ROUTE_SIZE = struct.calcsize(ROUTE_FORMAT)
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
TASK_RESULT_SIZE = struct.calcsize(TASK_RESULT_FORMAT)

STATE_NAMES = {0: "IDLE", 1: "PLANNING", 2: "MOVING", 3: "WAITING"}
STATE_COLORS = {0: (140, 140, 140), 1: (200, 200, 80), 2: (60, 180, 90), 3: (220, 60, 60)}
ROBOT_COLORS = {1: (60, 140, 220), 2: (230, 140, 40), 3: (160, 80, 200), 4: (200, 60, 120), 5: (80, 200, 200)}

STALE_AFTER_S = 6
LOG_MAX_LINES = 8

edges = [
    (1, 2), (1, 10), (2, 3), (3, 4), (4, 5), (5, 6), (6, 7), (7, 8), (3, 8), (8, 9),
    (9, 10), (10, 11), (11, 12), (12, 13), (13, 14), (8, 13), (14, 15), (15, 16),
    (16, 17), (17, 18), (15, 18), (18, 19), (19, 20), (11, 20),
]

LOG_HEIGHT = 140
MIN_WIDTH, MIN_HEIGHT = 760, 560


def compute_layout(width, height):
    """Recomputes node positions and rack rectangles proportionally to the
    current window size, so resizing the window rescales the whole floor
    plan instead of clipping it or leaving dead space."""
    graph_x0, graph_x1 = 360, width - 60
    graph_y0, graph_y1 = 100, height - LOG_HEIGHT - 30
    col_x = [graph_x0 + i * (graph_x1 - graph_x0) / 4 for i in range(5)]
    row_y = [graph_y0 + i * (graph_y1 - graph_y0) / 3 for i in range(4)]

    pos = {}
    rows = [[1, 2, 3, 4, 5], [10, 9, 8, 7, 6], [11, 12, 13, 14, 15], [20, 19, 18, 17, 16]]
    for row_i, row_nodes in enumerate(rows):
        for i, node in enumerate(row_nodes):
            pos[node] = (col_x[i], row_y[row_i])

    col_gap = (graph_x1 - graph_x0) / 4
    row_gap = (graph_y1 - graph_y0) / 3
    mx, my = col_gap * 0.25, row_gap * 0.3
    racks = [
        (col_x[0] - mx, row_y[0] + my, col_x[2] + mx, row_y[1] - my),
        (col_x[2] - mx, row_y[0] + my, col_x[4] + mx, row_y[1] - my),
        (col_x[0] - mx, row_y[1] + my, col_x[2] + mx, row_y[2] - my),
        (col_x[2] - mx, row_y[1] + my, col_x[4] + mx, row_y[2] - my),
        (col_x[0] - mx, row_y[2] + my, col_x[2] + mx, row_y[3] - my),
        (col_x[2] - mx, row_y[2] + my, col_x[4] + mx, row_y[3] - my),
    ]
    return pos, racks


positions, RACKS = compute_layout(1120, 860)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(("", UDP_PORT))
sock.setblocking(False)

robots = {}
event_log = []
selected_robot = None
current_mode = "smart"   # locally tracked assumption of global mode, for display only
auto_queue_on = False

# comparison stats: list of durations (ms) per mode
smart_durations = []
naive_durations = []


def log_event(text):
    timestamp = time.strftime("%H:%M:%S")
    event_log.append(f"[{timestamp}] {text}")
    if len(event_log) > LOG_MAX_LINES:
        event_log.pop(0)


def get_robot(robot_id):
    return robots.setdefault(robot_id, {
        "current_node": 1, "state": 0, "eta": 0.0, "last_seen": 0.0, "ip": None,
        "route": (), "wait_start": None, "total_wait": 0.0,
        "reroute_count": 0, "yield_count": 0, "task_count": 0,
    })


def send_task(robot_id, target_node):
    packet = struct.pack(TASK_CMD_FORMAT, MSG_TASK_CMD, robot_id, target_node)
    r = robots.get(robot_id)
    dest = (r["ip"], UDP_PORT) if r and r.get("ip") else (BROADCAST_IP, UDP_PORT)
    sock.sendto(packet, dest)
    print(f"[SENT] task -> robot {robot_id}, node {target_node}, to {dest}")


def send_graph_config():
    flat = []
    for a, b in edges[:MAX_EDGES]:
        flat.extend([a, b])
    flat += [0] * (MAX_EDGES * 2 - len(flat))
    packet = struct.pack(GRAPH_CONFIG_FORMAT, MSG_GRAPH_CONFIG, len(edges), *flat)
    sock.sendto(packet, (BROADCAST_IP, UDP_PORT))
    print(f"[SENT] graph config, {len(edges)} edges")
    log_event(f"Pushed graph config ({len(edges)} edges) to all robots")


def send_mode(smart: bool):
    global current_mode
    packet = struct.pack(MODE_CMD_FORMAT, MSG_MODE_CMD, 0, 1 if smart else 0)
    sock.sendto(packet, (BROADCAST_IP, UDP_PORT))
    current_mode = "smart" if smart else "naive"
    log_event(f"Mode set to {current_mode.upper()} for all robots")


def send_autoqueue(enabled: bool):
    global auto_queue_on
    packet = struct.pack(AUTOQ_CMD_FORMAT, MSG_AUTOQ_CMD, 0, 1 if enabled else 0)
    sock.sendto(packet, (BROADCAST_IP, UDP_PORT))
    auto_queue_on = enabled
    log_event(f"Auto-queue {'ON' if enabled else 'OFF'} for all robots")


def trigger_stress_test():
    """Force a guaranteed conflict: each known live robot is sent to the
    CURRENT node of the next robot in rotation, so their paths are certain
    to cross rather than hoping a conflict happens naturally."""
    live_ids = [rid for rid, r in robots.items() if time.time() - r["last_seen"] <= STALE_AFTER_S]
    if len(live_ids) < 2:
        log_event("Stress test needs at least 2 live robots — none/one found")
        return
    live_ids.sort()
    nodes = [robots[rid]["current_node"] for rid in live_ids]
    rotated = nodes[1:] + nodes[:1]
    for rid, target in zip(live_ids, rotated):
        send_task(rid, target)
    log_event(f"STRESS TEST: forced conflicting tasks across robots {live_ids}")


def handle_event(robot_id, event_type, node):
    r = get_robot(robot_id)
    if event_type == EVT_TASK_START:
        r["task_count"] += 1
        log_event(f"Robot {robot_id} new task -> node {node}")
    elif event_type == EVT_WAITING:
        r["wait_start"] = time.time()
        log_event(f"Robot {robot_id} waiting before node {node}")
    elif event_type == EVT_RESUMED:
        if r["wait_start"]:
            r["total_wait"] += time.time() - r["wait_start"]
            r["wait_start"] = None
        log_event(f"Robot {robot_id} resumed moving")
    elif event_type == EVT_REROUTED:
        r["reroute_count"] += 1
        if r["wait_start"]:
            r["total_wait"] += time.time() - r["wait_start"]
            r["wait_start"] = None
        log_event(f"Robot {robot_id} rerouted around node {node}")
    elif event_type == EVT_YIELDED:
        r["yield_count"] += 1
        log_event(f"Robot {robot_id} yielding -> stepping aside to node {node}")
    elif event_type == EVT_ARRIVED:
        if r["wait_start"]:
            r["total_wait"] += time.time() - r["wait_start"]
            r["wait_start"] = None
        log_event(f"Robot {robot_id} reached node {node}, now idle")


def handle_task_result(robot_id, target_node, mode, duration_ms):
    seconds = duration_ms / 1000.0
    mode_name = "SMART" if mode == 1 else "NAIVE"
    if mode == 1:
        smart_durations.append(duration_ms)
    else:
        naive_durations.append(duration_ms)
    log_event(f"Robot {robot_id} completed -> node {target_node} in {seconds:.1f}s [{mode_name}]")


def poll_network():
    while True:
        try:
            data, addr = sock.recvfrom(1024)
        except BlockingIOError:
            break
        if len(data) < 1:
            continue
        msg_type = data[0]

        if msg_type == MSG_HEARTBEAT and len(data) == HEARTBEAT_SIZE:
            _, robot_id, current_node, state, eta = struct.unpack(HEARTBEAT_FORMAT, data)
            r = get_robot(robot_id)
            r["current_node"] = current_node
            r["state"] = state
            r["eta"] = eta
            r["last_seen"] = time.time()
            r["ip"] = addr[0]

        elif msg_type == MSG_ROUTE and len(data) == ROUTE_SIZE:
            unpacked = struct.unpack(ROUTE_FORMAT, data)
            robot_id = unpacked[1]
            route = unpacked[2:2 + MAX_ROUTE_LEN]
            route_len = unpacked[2 + MAX_ROUTE_LEN]
            r = get_robot(robot_id)
            r["route"] = tuple(route[:route_len])

        elif msg_type == MSG_EVENT and len(data) == EVENT_SIZE:
            _, robot_id, event_type, node = struct.unpack(EVENT_FORMAT, data)
            handle_event(robot_id, event_type, node)

        elif msg_type == MSG_TASK_RESULT and len(data) == TASK_RESULT_SIZE:
            _, robot_id, target_node, mode, duration_ms = struct.unpack(TASK_RESULT_FORMAT, data)
            handle_task_result(robot_id, target_node, mode, duration_ms)

        # MSG_TASK_CMD / MSG_GRAPH_CONFIG / MSG_MODE_CMD / MSG_AUTOQ_CMD: dashboard-originated, ignore echoes.


pygame.init()
screen_width, screen_height = 1120, 860
screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)
pygame.display.set_caption("AutoNav Dashboard")
font = pygame.font.SysFont("consolas", 15)
small_font = pygame.font.SysFont("consolas", 13)
big_font = pygame.font.SysFont("consolas", 22, bold=True)

NODE_CLICK_RADIUS = 20
ROBOT_CLICK_RADIUS = 14


def draw_racks():
    for x1, y1, x2, y2 in RACKS:
        rect = pygame.Rect(x1, y1, x2 - x1, y2 - y1)
        pygame.draw.rect(screen, (45, 45, 55), rect, border_radius=4)
        pygame.draw.rect(screen, (100, 100, 115), rect, 2, border_radius=4)


def draw_graph():
    for a, b in edges:
        if a in positions and b in positions:
            pygame.draw.line(screen, (90, 90, 100), positions[a], positions[b], 3)
    for node, (x, y) in positions.items():
        ring_color = (255, 255, 0) if selected_robot is not None else (200, 200, 210)
        pygame.draw.circle(screen, (50, 50, 60), (int(x), int(y)), 16)
        pygame.draw.circle(screen, ring_color, (int(x), int(y)), 16, 2)
        label = font.render(str(node), True, (230, 230, 230))
        screen.blit(label, (x - 7, y - 9))


def draw_route_highlights():
    for robot_id, info in robots.items():
        if time.time() - info["last_seen"] > STALE_AFTER_S:
            continue
        route = info["route"]
        node = info["current_node"]
        if node not in route:
            continue
        idx = route.index(node)
        if idx + 1 >= len(route):
            continue
        color = ROBOT_COLORS.get(robot_id, (255, 255, 255))
        pts = [positions[n] for n in route[idx:] if n in positions]
        if len(pts) >= 2:
            pygame.draw.lines(screen, color, False, pts, 4)


def draw_robots():
    for robot_id, info in robots.items():
        if time.time() - info["last_seen"] > STALE_AFTER_S:
            continue
        node = info["current_node"]
        if node not in positions:
            continue
        x, y = positions[node]
        color = ROBOT_COLORS.get(robot_id, (255, 255, 255))
        state_color = STATE_COLORS.get(info["state"], (255, 255, 255))
        pygame.draw.circle(screen, color, (int(x), int(y)), 10)
        pygame.draw.circle(screen, state_color, (int(x), int(y)), 10, 3)
        if selected_robot == robot_id:
            pygame.draw.circle(screen, (255, 255, 0), (int(x), int(y)), 16, 2)


def draw_panel():
    x0, y = 20, 20
    screen.blit(big_font.render("AutoNav Dashboard", True, (255, 255, 255)), (x0, y))
    y += 30

    mode_color = (60, 200, 100) if current_mode == "smart" else (220, 100, 60)
    header = f"Mode: {current_mode.upper()}   Auto-queue: {'ON' if auto_queue_on else 'OFF'}"
    screen.blit(font.render(header, True, mode_color), (x0, y))
    y += 22

    hint = "Click robot then node to send  |  m=mode  q=auto-queue  t=stress test  g=push graph  c=clear stats"
    screen.blit(small_font.render(hint, True, (140, 140, 140)), (x0, y))
    y += 20

    if selected_robot is not None:
        screen.blit(font.render(f"Robot {selected_robot} selected — click a node", True, (255, 255, 0)), (x0, y))
    y += 22

    for robot_id in sorted(robots.keys()):
        info = robots[robot_id]
        stale = time.time() - info["last_seen"] > STALE_AFTER_S
        state_name = STATE_NAMES.get(info["state"], "?")
        color = ROBOT_COLORS.get(robot_id, (255, 255, 255))

        line1 = f"Robot {robot_id}: {state_name:8s} node:{info['current_node']:>3}  ETA:{info['eta']:.2f}s"
        if stale:
            line1 += "  (stale)"
        screen.blit(font.render(line1, True, color), (x0, y))
        y += 20

        wait_display = info["total_wait"] + (time.time() - info["wait_start"] if info["wait_start"] else 0)
        line2 = (f"   tasks:{info['task_count']}  reroutes:{info['reroute_count']}  "
                 f"yields:{info['yield_count']}  total wait:{wait_display:.1f}s")
        screen.blit(small_font.render(line2, True, (170, 170, 170)), (x0, y))
        y += 22

    # --- comparison stats: the actual PS success-criteria number ---
    y += 6
    screen.blit(font.render("Comparison (smart vs naive):", True, (220, 220, 220)), (x0, y))
    y += 20
    smart_avg = sum(smart_durations) / len(smart_durations) if smart_durations else None
    naive_avg = sum(naive_durations) / len(naive_durations) if naive_durations else None

    smart_line = f"  SMART: n={len(smart_durations)}  avg={smart_avg/1000:.2f}s" if smart_avg else "  SMART: no data yet"
    screen.blit(small_font.render(smart_line, True, (100, 220, 140)), (x0, y))
    y += 18
    naive_line = f"  NAIVE: n={len(naive_durations)}  avg={naive_avg/1000:.2f}s" if naive_avg else "  NAIVE: no data yet"
    screen.blit(small_font.render(naive_line, True, (220, 140, 100)), (x0, y))
    y += 18

    if smart_avg and naive_avg:
        improvement = (naive_avg - smart_avg) / naive_avg * 100
        diff_color = (100, 220, 140) if improvement > 0 else (220, 100, 100)
        screen.blit(font.render(f"  -> Smart is {improvement:.1f}% faster than naive", True, diff_color), (x0, y))
        y += 20


def draw_event_log():
    log_top = screen_height - LOG_HEIGHT
    pygame.draw.rect(screen, (18, 18, 22), (0, log_top, screen_width, LOG_HEIGHT))
    pygame.draw.line(screen, (80, 80, 90), (0, log_top), (screen_width, log_top), 2)
    screen.blit(font.render("Event log", True, (200, 200, 200)), (20, log_top + 8))
    y = log_top + 32
    for line in event_log[-6:]:
        screen.blit(small_font.render(line, True, (180, 200, 180)), (20, y))
        y += 18


def find_robot_at(pos):
    for robot_id, info in robots.items():
        if time.time() - info["last_seen"] > STALE_AFTER_S:
            continue
        node = info["current_node"]
        if node not in positions:
            continue
        rx, ry = positions[node]
        if (pos[0]-rx)**2 + (pos[1]-ry)**2 <= ROBOT_CLICK_RADIUS**2:
            return robot_id
    return None


def find_node_at(pos):
    for node, (nx, ny) in positions.items():
        if (pos[0]-nx)**2 + (pos[1]-ny)**2 <= NODE_CLICK_RADIUS**2:
            return node
    return None


send_graph_config()

running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.VIDEORESIZE:
            screen_width = max(event.w, MIN_WIDTH)
            screen_height = max(event.h, MIN_HEIGHT)
            screen = pygame.display.set_mode((screen_width, screen_height), pygame.RESIZABLE)
            positions, RACKS = compute_layout(screen_width, screen_height)
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = event.pos
            clicked_robot = find_robot_at(pos)
            if clicked_robot is not None:
                selected_robot = clicked_robot
            else:
                clicked_node = find_node_at(pos)
                if clicked_node is not None and selected_robot is not None:
                    send_task(selected_robot, clicked_node)
                    selected_robot = None
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                selected_robot = None
            elif event.key == pygame.K_g:
                send_graph_config()
            elif event.key == pygame.K_m:
                send_mode(current_mode != "smart")  # toggle
            elif event.key == pygame.K_q:
                send_autoqueue(not auto_queue_on)
            elif event.key == pygame.K_t:
                trigger_stress_test()
            elif event.key == pygame.K_c:
                smart_durations.clear()
                naive_durations.clear()
                log_event("Comparison stats cleared")

    poll_network()

    screen.fill((25, 25, 30))
    draw_racks()
    draw_graph()
    draw_route_highlights()
    draw_robots()
    draw_panel()
    draw_event_log()
    pygame.display.flip()
    clock.tick(30)

pygame.quit()
sock.close()
