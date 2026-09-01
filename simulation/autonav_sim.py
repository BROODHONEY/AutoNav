"""
FIXED decision logic: physical occupation of a node is now an unconditional
block regardless of priority. Priority only resolves races for an EMPTY
contested node. A mutual head-on's loser, when it can't reroute (because
its own destination IS the contested node), now takes a temporary sidestep
instead of freezing in place forever — which is what actually clears the
node for the winner.
"""
from collections import deque

EDGES = [
    (1,2),(1,10),(2,3),(3,4),(4,5),(5,6),(6,7),(7,8),(3,8),(8,9),
    (9,10),(10,11),(11,12),(12,13),(13,14),(8,13),(14,15),(15,16),
    (16,17),(17,18),(18,13),(18,19),(19,20),(11,20),(6,15),
]
MAX_NODES = 20
ADJ = {n: set() for n in range(1, MAX_NODES+1)}
for a, b in EDGES:
    ADJ[a].add(b); ADJ[b].add(a)

OCCUPIED_PENALTY = 6
IDLE_YIELD_GRACE_TICKS = 30
STARVATION_OVERRIDE_TICKS = 60
SIDESTEP_WAIT_TICKS = 15
TRAVEL_TICKS = 5
IDLE, PLANNING, MOVING, WAITING = 0, 1, 2, 3

class Robot:
    def __init__(self, rid, start):
        self.id = rid
        self.route = [start]; self.idx = 0
        self.state = IDLE
        self.speed_ticks = TRAVEL_TICKS
        self.waiting_since = None
        self.travel_until = None
        self.plan_settle_until = None
        self.real_destination = None   # remembered across a temporary sidestep
        self.events = []
    @property
    def node(self): return self.route[self.idx]
    def next_node(self): return self.route[self.idx+1] if self.idx+1 < len(self.route) else None
    def snapshot(self): return {"id": self.id, "node": self.node, "next": self.next_node(),
                                 "state": self.state, "route": list(self.route)}
    def log(self, tick, text): self.events.append((tick, f"R{self.id}: {text}"))

def is_occupied_snap(snaps, me_id, node):
    return any(s["node"] == node for s in snaps if s["id"] != me_id)

def is_needed_by_another_snap(snaps, me_id, node, except_id):
    for s in snaps:
        if s["id"] == me_id: continue
        if s["id"] != except_id and s["next"] == node: return True
        if s["route"] and s["route"][-1] == node: return True
    return False

def dijkstra_smart_snap(snaps, me_id, start, target):
    dist = {n: float('inf') for n in range(1, MAX_NODES+1)}
    parent = {}; dist[start] = 0; visited = set()
    while True:
        u = min((n for n in dist if n not in visited), key=lambda n: dist[n], default=None)
        if u is None or dist[u] == float('inf'): break
        visited.add(u)
        if u == target: break
        for v in ADJ[u]:
            cost = 1 + (OCCUPIED_PENALTY if is_occupied_snap(snaps, me_id, v) else 0)
            if dist[u] + cost < dist[v]: dist[v] = dist[u] + cost; parent[v] = u
    if dist[target] == float('inf'): return None
    path = [target]
    while path[-1] != start: path.append(parent[path[-1]])
    return list(reversed(path))

def bfs_avoiding(start, target, avoid):
    visited = {avoid, start}; parent = {}; q = deque([start])
    while q:
        u = q.popleft()
        if u == target: break
        for v in ADJ[u]:
            if v not in visited: visited.add(v); parent[v]=u; q.append(v)
    if target not in visited or target == avoid: return None
    path = [target]
    while path[-1] != start: path.append(parent[path[-1]])
    return list(reversed(path))

def has_priority(a_id, b_id): return a_id < b_id

def find_occupant(snaps, me, node):
    for s in snaps:
        if s["id"] != me.id and s["node"] == node: return s
    return None

def find_racer(snaps, me, node):
    for s in snaps:
        if s["id"] != me.id and s["node"] != node and s["next"] == node: return s
    return None

def decide(me, snaps, tick):
    action = {"robot": me}

    if me.state == IDLE:
        for s in snaps:
            if s["id"] == me.id or s["state"] == IDLE: continue
            if s["next"] != me.node: continue
            for n in sorted(ADJ[me.node]):
                if is_occupied_snap(snaps, me.id, n): continue
                if is_needed_by_another_snap(snaps, me.id, n, s["id"]): continue
                action["type"]="yield"; action["target"]=n; action["blocker"]=s["id"]
                return action
        return action

    if me.state == PLANNING and tick >= me.plan_settle_until:
        action["type"]="start_moving"; return action

    if not (me.state == MOVING and tick >= (me.travel_until or 0)) and me.state != WAITING:
        return action

    if me.idx + 1 >= len(me.route):
        action["type"]="arrive"; return action

    nxt = me.next_node()

    # ---- Rule 1: physical occupation is an unconditional block, no matter
    # who has "priority". Priority is not a license to walk through a node
    # someone is standing on. ----
    occupant = find_occupant(snaps, me, nxt)
    if occupant is not None:
        if occupant["state"] == IDLE:
            if me.waiting_since is None or tick - me.waiting_since < IDLE_YIELD_GRACE_TICKS:
                if me.waiting_since is None: me.waiting_since = tick
                action["type"]="wait"; return action
            alt = bfs_avoiding(me.node, me.route[-1], nxt)
            if alt:
                action["type"]="reroute"; action["path"]=alt; action["reason"]=f"idle R{occupant['id']} not moving"
                return action
            action["type"]="wait"; return action

        # occupant is live and moving/waiting — I must wait regardless of
        # priority; if it's a mutual head-on and I'm the one who should
        # yield, try to actually clear space (reroute, or temp sidestep).
        mutual = (occupant["next"] == me.node)
        i_have_priority = has_priority(me.id, occupant["id"])
        if mutual and not i_have_priority:
            alt = bfs_avoiding(me.node, me.route[-1], nxt)
            if alt:
                action["type"]="reroute"; action["path"]=alt; action["reason"]=f"head-on with R{occupant['id']}"
                return action
            # No reroute possible (my own destination IS the contested
            # node) — take a TEMPORARY sidestep instead of freezing in
            # place, which is what actually lets the winner through.
            for n in sorted(ADJ[me.node]):
                if n == nxt: continue
                if is_occupied_snap(snaps, me.id, n): continue
                if is_needed_by_another_snap(snaps, me.id, n, occupant["id"]): continue
                action["type"]="sidestep"; action["target"]=n; action["real_dest"]=me.route[-1]
                return action
            action["type"]="wait"; return action

        # Not mutual, or I have priority: I still cannot enter an occupied
        # node — just wait for the occupant to actually leave.
        if me.waiting_since is None: me.waiting_since = tick
        waited = tick - me.waiting_since
        if waited < STARVATION_OVERRIDE_TICKS:
            action["type"]="wait"; return action
        alt = bfs_avoiding(me.node, me.route[-1], nxt)
        if alt:
            action["type"]="reroute"; action["path"]=alt; action["reason"]="starvation"
            return action
        action["type"]="wait"; return action

    # ---- Rule 2: node is empty right now — only a RACE (both want it,
    # neither is there yet) is resolved by priority. ----
    racer = find_racer(snaps, me, nxt)
    if racer is not None and not has_priority(me.id, racer["id"]):
        if me.waiting_since is None: me.waiting_since = tick
        waited = tick - me.waiting_since
        if waited < STARVATION_OVERRIDE_TICKS:
            action["type"]="wait"; return action
        alt = bfs_avoiding(me.node, me.route[-1], nxt)
        if alt:
            action["type"]="reroute"; action["path"]=alt; action["reason"]="starvation(race)"
            return action
        action["type"]="wait"; return action

    action["type"]="advance"
    return action

def apply(action, tick):
    me = action["robot"]; t = action.get("type")
    if t == "yield":
        me.route, me.idx = [me.node, action["target"]], 0
        me.state = MOVING; me.travel_until = tick + me.speed_ticks
        me.log(tick, f"yields to R{action['blocker']} -> steps aside to {action['target']}")
    elif t == "sidestep":
        me.real_destination = action["real_dest"]
        me.route, me.idx = [me.node, action["target"]], 0
        me.waiting_since = None
        me.state = MOVING; me.travel_until = tick + me.speed_ticks
        me.log(tick, f"temp sidestep to {action['target']} (real dest {action['real_dest']} still pending)")
    elif t == "start_moving":
        me.state = MOVING; me.travel_until = tick + me.speed_ticks
    elif t == "arrive":
        if me.real_destination is not None and me.node != me.real_destination:
            # arrived at the sidestep node, not the real destination — plan
            # the resumed leg the SAME way the firmware would (occupancy-
            # aware) once we've stepped out of the way.
            me.log(tick, f"cleared sidestep, will resume toward {me.real_destination}")
            me._resume_target = me.real_destination
            me.real_destination = None
            me.state = "NEEDS_REPLAN"
        else:
            me.state = IDLE
    elif t == "reroute":
        me.route, me.idx = action["path"], 0
        me.waiting_since = None
        me.state = MOVING; me.travel_until = tick + me.speed_ticks
        me.log(tick, f"{action['reason']} -> reroutes {action['path']}")
    elif t == "wait":
        if me.waiting_since is None: me.waiting_since = tick
        me.state = WAITING
    elif t == "advance":
        was_waiting = me.waiting_since is not None
        me.waiting_since = None
        me.idx += 1
        me.state = IDLE if me.idx+1 >= len(me.route) else MOVING
        me.travel_until = tick + me.speed_ticks
        if was_waiting: me.log(tick, f"resumed, entered node {me.node}")

def run(name, robots, max_ticks=300, verbose=False):
    collisions = []
    for tick in range(max_ticks):
        # handle any pending replans from a completed sidestep
        for r in robots:
            if r.state == "NEEDS_REPLAN":
                snaps = [x.snapshot() for x in robots]
                path = dijkstra_smart_snap(snaps, r.id, r.node, r._resume_target)
                if path and len(path) > 1:
                    r.route, r.idx = path, 0
                    r.state = PLANNING
                    r.plan_settle_until = tick + 2
                    r.log(tick, f"replans after sidestep -> {path}")
                else:
                    r.state = IDLE

        snaps = [r.snapshot() for r in robots]
        actions = [decide(r, snaps, tick) for r in robots]
        for a in actions:
            apply(a, tick)

        nodes = [r.node for r in robots]
        if len(nodes) != len(set(nodes)):
            collisions.append((tick, [r.id for r in robots if nodes.count(r.node) > 1], list(nodes)))

    print(f"\n=== {name} ===")
    if verbose:
        allev = sorted((e for r in robots for e in r.events), key=lambda e: e[0])
        for t, msg in allev:
            print(f"  t={t:3d}  {msg}")
    print("Final positions:", {r.id: r.node for r in robots}, "states:", {r.id: r.state for r in robots})
    if collisions:
        print(f"  *** COLLISION DETECTED *** : {collisions[:5]}")
    else:
        print("  No collisions detected.")
    stuck = [r.id for r in robots if r.state == WAITING and r.idx+1 < len(r.route)]
    print("  Still WAITING at end:", stuck if stuck else "none")
    return robots, collisions
