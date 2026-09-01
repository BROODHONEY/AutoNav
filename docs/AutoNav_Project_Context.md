# AutoNav — Project Context Brief
*(For use with other AI tools generating images, diagrams, or slides)*

## 1. Competition Context

- **Event**: Smart India Hackathon (SIH) 2026
- **Problem Statement ID**: 26123
- **Problem Statement Title**: "Edge-AI Based Distributed Fleet Coordination for Autonomous Mobile Robots (AMRs) in Smart Warehouses"
- **PS Category**: Software
- **Theme**: Robotics and Drones
- **Project/Idea Name**: **AutoNav**
- **Tagline**: Decentralized Edge-AI Fleet Coordination for Autonomous Mobile Robots in Smart Warehouses

## 2. The Problem

Warehouse AGVs/AMRs typically plan routes blindly, without accounting for other robots. Fixed priority rules cause avoidable waiting, and centralized fleet controllers are a single point of failure. As fleets scale up, this produces collisions, deadlocks, and downtime. Most real-world warehouse robots today are actually **graph-constrained AGVs** (guided-path navigation along a fixed network of nodes/aisles), not free-roaming SLAM-based robots — AutoNav embraces this honestly rather than overclaiming free-roam autonomy.

## 3. The Solution — What AutoNav Actually Is

A **fully decentralized fleet-coordination system** — no central server, no single point of failure. Each robot is an independent ESP32 microcontroller that:

1. Runs on a fixed **topological graph** (nodes = intersections/pickup-dropoff points, edges = aisles) rather than continuous coordinates — matches real guided-path AGV navigation.
2. Autonomously claims **jobs** (pickup node → dropoff node pairs) from a shared, decentralized job pool — lowest job ID among pending jobs wins, ties resolved deterministically by robot ID. Jobs can also be manually reserved for a specific robot.
3. Plans routes using an **occupancy-aware Dijkstra variant** — nodes currently occupied by another robot are treated as *expensive* (a penalty), not blocked outright, so the robot naturally routes around congestion when a reasonable detour exists, without making anything truly unreachable.
4. Uses a **TinyML neural network** (trained offline, hand-rolled inference on-device — no heavy framework needed) to predict each robot's ETA to its next node. This ETA is the deciding factor in right-of-way conflicts — not a fixed priority rule.
5. Follows a **Plan → Confirm → Move** sequence: claiming a job puts the robot in a `PLANNING` state that broadcasts its intended route and holds briefly before actually moving, giving peers a chance to react.
6. Resolves conflicts through a layered decentralized protocol (see Section 5).
7. Communicates entirely via **WiFi UDP broadcast** with a custom typed binary protocol — no router-dependent infrastructure beyond a shared WiFi network.

## 4. Hardware / Physical Setup

- **3x ESP32 microcontrollers** (WROOM-32), one per robot — chosen for built-in WiFi, low cost (~₹380/unit), and enough compute for the graph algorithms and TinyML inference.
- No physical chassis/motors were built in the final phase — the project pivoted to a **software-simulated movement model** (a timer, driven by the TinyML ETA prediction, advances each robot's position along the graph) running on **real embedded hardware** doing **real decentralized coordination** — this is a deliberate, honest framing: the coordination "brain" is 100% real distributed embedded software; physical wheels/motion are simulated for demo practicality.
- Warehouse layout: a **20-node graph** arranged as a 4-row × 5-column grid, with the node numbering following a snake/boustrophedon pattern (row 1 left-to-right, row 2 right-to-left, etc.), matching a real hand-drawn warehouse floor plan with 6 rectangular rack/shelf blocks positioned between the rows (3 rows of racks, 2 racks per row). Nodes are connected by three full vertical "cross-aisles" (at the left edge, center, and right edge columns) plus horizontal aisle chains within each row.

## 5. The Deadlock/Conflict-Resolution Stack (the core technical contribution)

Built and hardened iteratively, including bugs found via rigorous simulation before hardware testing:

- **Occupation is an unconditional block.** A currently-occupied node can never be entered by another robot regardless of priority — this was a real bug found and fixed (priority was originally, incorrectly, letting a "winning" robot walk into a still-occupied node in head-on scenarios).
- **Priority only resolves races for empty nodes** — when two robots are both racing toward a node neither currently occupies, the one with the lower predicted ETA (from the TinyML model) goes first; ties broken by lower robot ID.
- **Idle-yield**: a parked (idle) robot that's blocking another robot's path proactively steps aside to a free neighboring node — but only if that neighbor isn't itself needed by anyone else's route or final destination (a subtler bug: the first version let a robot yield directly into the very node the *other* robot was trying to reach).
- **Head-on reroute**: if two moving robots want each other's current node (a classic swap conflict), the lower-priority one reroutes around the contested node using a hard-exclude BFS.
- **Temporary sidestep**: if a head-on's loser *can't* reroute (because the contested node is literally its own destination — a true bottleneck swap), it takes a temporary detour to a third node, waits, then resumes toward its real destination once the path clears. This is what actually resolves the "impossible" case that reroute logic alone can't fix.
- **Starvation override with staleness check**: a robot that's waited past a threshold gets priority regardless of the normal rule — but only if the blocking peer has gone silent/stale (dead/disconnected), never against a live, legitimately-there peer. (An earlier, unsafe version overrode unconditionally after a timeout — fixed.)
- **Config-driven graph**: the warehouse layout can be pushed to all robots at runtime from the dashboard — no firmware re-flash needed to change the floor plan.

All of this was validated via a **from-scratch Python simulation** that faithfully ported the firmware's decision logic and tested 7+ adversarial scenarios (head-on collisions, idle blockers, circular 3-robot contention, hub congestion, starvation, and the hardest case — a true corridor swap deadlock with no nearby branch) — all passing with zero collisions after fixes.

## 6. Communications Protocol

Custom lightweight binary protocol over WiFi UDP broadcast, every message prefixed with a type byte:
- **Heartbeat** (8 bytes) — frequent, cheap: current node, state, ETA
- **Route** — sent only when a route changes (not every heartbeat) — full path array
- **Event** — explicit typed events: WAITING, RESUMED, REROUTED, YIELDED, JOB_CLAIMED, ARRIVED_PICKUP, ARRIVED_DROPOFF
- **JobAnnounce / JobClaim / JobComplete / JobResult** — the decentralized job lifecycle
- **GraphConfig** — pushes the warehouse layout
- **ModeCommand** — toggles "smart" vs "naive" (a plain stop-and-wait baseline) fleet-wide, for quantified comparison

## 7. Dashboard

A **web-based dashboard** (HTML/CSS/JS frontend with SVG graph rendering + a Python asyncio/WebSocket bridge that speaks the same UDP protocol as the robots):
- Live SVG map of the warehouse graph with rack shelving drawn in, robots shown as colored dots that animate smoothly between nodes
- Each moving robot's remaining route highlighted as a colored line
- Job list panel: PENDING / CLAIMED (by robot X) / DELIVERED, with support for jobs reserved for a specific robot
- Event log (left panel) — live feed of waits, reroutes, yields, sidesteps, deliveries
- Per-robot stats: task count, reroute count, yield count, cumulative wait time
- **Smart vs Naive comparison panel** — runs both modes and shows live average job-completion time and % improvement, directly answering the PS's success criterion
- Click-to-create jobs (click pickup node, then dropoff node) or a typed command box (`job 10 20`, `assign 2 10 20`, `random`, `mode smart`, `graph`)
- **Collision watchdog** — flags in real time (banner + flashing robot dots) if two robots ever report the same node simultaneously — should never fire; exists as a live safety check

## 8. Visual / Brand Style Already Established (for consistency)

- **Primary color**: `#0070C0` (blue) — inherited from the mandatory SIH template's own accent color
- **Secondary/dark**: `#1F3864` (navy)
- **Accent**: `#E97A2E` (orange), used sparingly for innovation/energy-themed highlights
- **Supporting**: green `#2E7D32` (positive/mitigation), red `#C0392B` (problem/risk/collision)
- **Card style**: light blue background (`#EAF2FB`), rounded rectangles, no accent stripes/bars (deliberately avoided — reads as "AI-generated template" if overused)
- **Diagrams**: workflow shown as a horizontal flowchart — rounded-terminator start/end nodes, rounded-rectangle process steps, diamond decision node, with a distinct branch color (red) for the exception/conflict path
- **Icon language**: simple, recognizable pictographic shapes rather than photographic images — e.g., a "no-entry" circle for problems, a gear for solutions/mechanisms, a lightning bolt for innovation/edge-AI, a cube for embedded hardware, a cloud for wireless/networking, a donut/ring shape for "no single point of failure," a star for milestones/achievement

## 9. Slide-by-Slide Content Already Built (SIH 6-slide format)

1. **Title** — PS ID/title/theme/category as above
2. **Proposed Solution** — three cards: Problem Statement / Proposed Solution / Innovation-UVP (content per Sections 2–3 above)
3. **Technical Approach** — the workflow diagram (Job Announced → Robot Claims [lowest-ID, decentralized] → Smart Planning [occupancy-aware + TinyML] → Conflict? → [No: Pickup→Dwell→Dropoff | Yes: Reroute/Yield/Sidestep] → Delivered) + a 5-item tech stack row: ESP32, WiFi/UDP, TinyML, Python+WebSocket bridge, Web Dashboard
4. **Feasibility and Viability** — milestone strip (Simulated in Python → Prototyped on ESP32 hardware → Field-tested over real WiFi) + paired Risk→Mitigation rows (WiFi packet loss → redundant sends; hardware cost → cheap ESP32; coordination latency → proactive occupancy-aware planning)
5. **Impact and Benefits** — fewer collisions/downtime, faster fulfillment (quantified live via the smart/naive comparison mode), scales without redesign, no single point of failure, applicability beyond warehouses (hospitals, airports, factories)
6. **Research and References** — Dijkstra's algorithm, TensorFlow Lite Micro, Espressif ESP-IDF/Arduino-ESP32 networking docs, decentralized multi-robot task allocation literature, warehouse AGV traffic-control industry practice

## 10. Notes for Whoever Generates Images/Slides Next

- Keep the **AGV/graph-constrained framing** honest — this is not a free-roaming SLAM robot, and overstating that would be inaccurate to what was actually built and tested.
- The workflow diagram (Section 3/9-slide-3) is the single most important visual to render well — it's the crux of the technical pitch.
- If generating a warehouse floor illustration: 4 rows × 5 columns of aisle nodes, 6 rack blocks in a 3×2 grid between the rows, three vertical cross-aisles.
- Any "before/after" or comparison visual should reflect: **naive = fixed stop-and-wait, prone to deadlock; smart = occupancy-aware planning + TinyML priority + self-healing reroute/sidestep.**
