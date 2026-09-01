# AutoNav

**Decentralized Edge-AI Fleet Coordination for Autonomous Mobile Robots in Smart Warehouses**

Built for Smart India Hackathon 2026 — Problem Statement 26123 (Software track, Robotics and Drones).

---

## What this is

AutoNav is a fleet-coordination system for warehouse AMRs where **no central server plans anything**. Each robot is an independent ESP32 that:

- Claims pickup → dropoff jobs from a shared, decentralized job pool
- Plans its own route on a shared warehouse graph using an **occupancy-aware Dijkstra variant** — congested nodes cost more, they aren't blocked outright
- Uses a small **on-device neural network (TinyML)** to predict its own travel time, which decides right-of-way at contested nodes instead of a fixed priority rule
- Resolves every real conflict case autonomously: reroute around a blocked node, yield/step-aside if parked in someone's way, or take a **temporary sidestep** for the one case reroute can't solve — a true head-on swap where the contested node is the robot's own destination
- Talks to its peers entirely over WiFi/UDP broadcast, with a custom lightweight binary protocol

A Python bridge + web dashboard listens on the same network to visualize the fleet live, create jobs, and quantify how much better the "smart" coordination logic performs against a plain stop-and-wait baseline.

**On scope, honestly:** this is a graph-constrained AGV-style system (fixed node/edge navigation), not a free-roaming SLAM-based robot — which matches how most real deployed warehouse robots actually work, and matches what the PS asks for (fleet *coordination*, not mapping). Physical movement between nodes is currently timer-driven (paced by the TinyML ETA prediction) rather than backed by real motors/wheels — the coordination logic itself runs as real distributed code on real embedded hardware. See [Limitations](#known-limitations-read-this) for the full honest picture, including a network-layer caveat worth knowing before you present this.

---

## Key features

- **Fully decentralized job claiming** — lowest-ID-first with deterministic tie-breaks; jobs can also be reserved for a specific robot
- **Occupancy-aware path planning** — routes around congestion proactively, not reactively
- **TinyML-driven priority** — a trained 2-8-1 neural net predicts ETA; no fixed priority rule
- **Layered deadlock resolution** — reroute, idle-yield, temporary sidestep, and a staleness-gated starvation override, each validated against adversarial scenarios (see [`/simulation`](./simulation))
- **Plan → Confirm → Move protocol** — a robot announces its route and briefly holds before committing
- **Live web dashboard** — SVG fleet map, job list, event log, per-robot stats, and a live Smart-vs-Naive throughput comparison
- **Config-driven graph** — push a new warehouse layout to every robot at runtime, no re-flash needed
- **Collision watchdog** — the dashboard flags in real time if two robots ever report the same node simultaneously

---

## Repository structure

```
AutoNav/
├── firmware/
│   └── autonav_integrated/
│       └── autonav_integrated.ino   # Main robot firmware (flash on every ESP32)
├── dashboard/
│   ├── autonav_bridge.py            # Python asyncio/WebSocket <-> UDP bridge
│   ├── autonav_dashboard.html       # Web frontend (open directly in a browser)
│   └── legacy_pygame_dashboard/     # Earlier pygame-based dashboard, superseded
├── ml/
│   └── train_tiny_mlp.py            # Offline trainer for the on-device ETA model
├── simulation/
│   ├── autonav_sim.py               # Python port of the firmware's decision logic
│   └── run_scenarios.py             # 7 adversarial deadlock scenarios (pre-hardware validation)
├── tests/
│   ├── esp_now_broadcast_test/      # Early ESP-NOW comms test (not used in final build)
│   ├── wifi_signal_test/            # WiFi radio bring-up test
│   ├── graph_bfs_test/              # Standalone path-planning test
│   ├── connection_test/             # Basic board bring-up / heartbeat test
│   ├── tinyml_eta_test/             # On-device model inference test
│   └── wifi_udp_hi_test/            # First WiFi/UDP two-board comms test
└── docs/
    ├── AutoNav_Project_Context.md          # Full project brief
    └── AutoNav_SIH_Idea_Presentation.pptx  # SIH idea submission deck
```

---

## Hardware required

- 3× ESP32 DevKit boards (WROOM-32) — one per robot
- USB cables for flashing
- A shared WiFi network that all boards and your dashboard laptop can join (same subnet, broadcast traffic allowed — see the note on client isolation below)

No motors, sensors, or chassis are required to run the coordination logic as-is — see [Limitations](#known-limitations-read-this).

---

## Getting started

### 1. Flash the firmware

Open `firmware/autonav_integrated/autonav_integrated.ino` in the Arduino IDE (with ESP32 board support installed). Before uploading to **each** board, edit these lines:

```cpp
#define ROBOT_ID 1                 // unique per board: 1, 2, or 3
const char* WIFI_SSID = "...";
const char* WIFI_PASSWORD = "...";
my_route[0] = 1;                   // that robot's real starting node
```

Upload to all three boards with different `ROBOT_ID` values.

### 2. Run the dashboard bridge

```bash
cd dashboard
pip install websockets --break-system-packages
python autonav_bridge.py
```

Keep this running — it joins the same WiFi network as the robots and relays everything to the browser.

### 3. Open the dashboard

Double-click `dashboard/autonav_dashboard.html`. It connects automatically to `ws://localhost:8765`. If your browser blocks the `file://` WebSocket connection, serve it instead:

```bash
python -m http.server
# then open http://localhost:8000/autonav_dashboard.html
```

### 4. Create a job

Click a node (pickup), then another (dropoff) on the map — or use the command box:

```
job 10 20          # open job, any robot may claim it
assign 2 10 20     # reserved specifically for robot 2
random             # random pickup/dropoff pair
mode smart | naive # toggle the coordination logic on/off fleet-wide
graph              # re-push the current warehouse layout to all robots
```

---

## Testing and validation

Before any of this ran on real hardware, the full decision logic (occupancy checks, priority resolution, reroute, sidestep) was ported to Python and tested against **7 adversarial scenarios** — head-on collisions, idle blockers, 3-robot circular contention, hub congestion, repeated starvation, and a worst-case corridor swap deadlock with no nearby detour. See [`simulation/run_scenarios.py`](./simulation/run_scenarios.py). This caught and fixed a real collision bug (priority was incorrectly overriding physical occupation) before it ever reached hardware.

---

## Known limitations (read this)

Being upfront about these matters more than pretending they don't exist:

- **The WiFi router is a real single point of failure at the network layer**, even though no *decision-making* is centralized. If the router goes down, every robot loses contact with every other robot simultaneously. "Decentralized" here describes the coordination logic, not the communication infrastructure. ESP-NOW (tested early, not used in the final build — see `tests/esp_now_broadcast_test/`) would remove this dependency but loses the dashboard's simple network-join integration.
- **Movement is currently simulated**, timed by the TinyML ETA prediction rather than driven by real motors — the coordination logic itself is real, distributed, embedded software.
- **The TinyML model is trained on synthetic data** (`distance/speed` with noise), not measurements from physical motion.
- **This is a graph-constrained AGV model**, not free-roaming SLAM-based navigation. See `docs/AutoNav_Project_Context.md` for the reasoned path to a true AMR (LiDAR/depth sensing, SLAM, Nav2-style continuous-space planning) — a legitimate next phase, not something attempted here.

---

## Tech stack

ESP32 (Arduino framework) · WiFi/UDP · Custom binary protocol · Hand-rolled on-device neural network inference · Python (asyncio, websockets) · HTML/CSS/JS + SVG

---

## License

MIT — see [`LICENSE`](./LICENSE).

## Team

Gradient Ascent — Smart India Hackathon 2026
