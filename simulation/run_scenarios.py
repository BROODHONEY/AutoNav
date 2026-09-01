from autonav_sim3 import Robot, run, dijkstra_smart_snap, MOVING

def head_on():
    r1=Robot(1,8); r1.route=[8,9]; r1.state=MOVING; r1.travel_until=0
    r2=Robot(2,9); r2.route=[9,8]; r2.state=MOVING; r2.travel_until=0
    r3=Robot(3,15)
    return [r1,r2,r3]

def idle_blocker():
    r1=Robot(1,4); r1.route=[4,5,6]; r1.state=MOVING; r1.travel_until=0
    r2=Robot(2,5)
    r3=Robot(3,15)
    return [r1,r2,r3]

def yield_into_destination():
    r1=Robot(1,10)
    r2=Robot(2,9); r2.route=[9,10,1]; r2.state=MOVING; r2.travel_until=0
    r3=Robot(3,20)
    return [r1,r2,r3]

def circular():
    r1=Robot(1,1); r1.route=[1,2]; r1.state=MOVING; r1.travel_until=0
    r2=Robot(2,2); r2.route=[2,3]; r2.state=MOVING; r2.travel_until=0
    r3=Robot(3,3); r3.route=[3,1,10]; r3.state=MOVING; r3.travel_until=0
    return [r1,r2,r3]

def three_way_hub():
    r1=Robot(1,3); r1.route=[3,8,13]; r1.state=MOVING; r1.travel_until=0
    r2=Robot(2,9); r2.route=[9,8,7]; r2.state=MOVING; r2.travel_until=0
    r3=Robot(3,8)
    return [r1,r2,r3]

def repeated_hot_junction():
    r1=Robot(1,3); r1.route=[3,8,13]; r1.state=MOVING; r1.travel_until=0
    r2=Robot(2,9); r2.route=[9,8]; r2.state=MOVING; r2.travel_until=1
    r3=Robot(3,7); r3.route=[7,8]; r3.state=MOVING; r3.travel_until=2
    return [r1,r2,r3]

def true_corridor_deadlock():
    # A genuinely bottlenecked swap: both destinations ARE the contested
    # node, on a path with no useful side branch nearby. Hardest case.
    r1=Robot(1,17); r1.route=[17,18]; r1.state=MOVING; r1.travel_until=0
    r2=Robot(2,18); r2.route=[18,17]; r2.state=MOVING; r2.travel_until=0
    r3=Robot(3,1)
    return [r1,r2,r3]

results = {}
for name, setup in [
    ("Head-on collision (THE bug)", head_on),
    ("Idle robot blocking a path", idle_blocker),
    ("Yield-into-destination regression", yield_into_destination),
    ("3-robot circular contention", circular),
    ("3-way simultaneous contention at a hub", three_way_hub),
    ("Repeated hot-junction (starvation check)", repeated_hot_junction),
    ("True corridor swap deadlock (hardest case)", true_corridor_deadlock),
]:
    robots, collisions = run(name, setup(), max_ticks=250, verbose=True)
    results[name] = len(collisions) == 0

print("\n\n========== SUMMARY ==========")
for name, ok in results.items():
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
