// AutoNav — firmware v3: decentralized job system (pickup -> dropoff)
//
//
// SETUP: change ROBOT_ID (1..5) and WIFI_SSID/WIFI_PASSWORD per board.
//
// SERIAL COMMANDS (for standalone testing without the dashboard):
//   j <pickup> <dropoff>  -> announce and locally queue a new job
//   m                     -> toggle smart / naive mode
//   <number>              -> debug-only: move directly to that node (bypasses jobs)
//   n                     -> manual override: force an immediate advance attempt
//   s <value>             -> set this robot's current speed

#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_system.h>

#define ROBOT_ID 1   // <-- CHANGE per board: 1..5
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const unsigned int UDP_PORT = 4210;
WiFiUDP udp;

#define MAX_NODES 40
#define MAX_ROUTE_LEN 20
#define MAX_ROBOTS 6
#define MAX_EDGES 60
#define MAX_JOBS 20
#define INF 9999

enum RobotState : uint8_t { IDLE=0, PLANNING=1, MOVING=2, WAITING=3 };
enum JobPhase : uint8_t { PHASE_NONE=0, PHASE_TO_PICKUP=1, PHASE_DWELL_PICKUP=2, PHASE_TO_DROPOFF=3 };
enum JobStatus : uint8_t { JOB_PENDING=0, JOB_CLAIMED=1, JOB_DELIVERED=2 };

enum MsgType : uint8_t {
  MSG_HEARTBEAT = 1,
  MSG_ROUTE = 2,
  MSG_EVENT = 3,
  MSG_GRAPH_CONFIG = 4,
  MSG_MODE_CMD = 5,
  MSG_JOB_ANNOUNCE = 6,
  MSG_JOB_CLAIM = 7,
  MSG_JOB_COMPLETE = 8,
  MSG_JOB_RESULT = 9
};

enum EventType : uint8_t {
  EVT_WAITING = 1,
  EVT_RESUMED = 2,
  EVT_REROUTED = 3,
  EVT_YIELDED = 4,
  EVT_JOB_CLAIMED = 5,
  EVT_ARRIVED_PICKUP = 6,
  EVT_ARRIVED_DROPOFF = 7
};

// ---------- Graph (default fallback; can be replaced at runtime by a
// GraphConfig message from the dashboard, with no re-flash needed) ----------
struct Edge { uint8_t a; uint8_t b; };
Edge defaultEdges[] = {
  {1,2}, {1,10}, {2,3}, {3,4}, {4,5}, {5,6}, {6,7}, {7,8}, {3,8}, {8,9},
  {9,10}, {10,11}, {11,12}, {12,13}, {13,14}, {8,13}, {14,15}, {15,16},
  {16,17}, {17,18}, {18,13}, {18,19}, {19,20}, {11,20}, {6,15}
};
const int numDefaultEdges = sizeof(defaultEdges) / sizeof(defaultEdges[0]);
uint8_t adjacency[MAX_NODES+1][MAX_NODES+1];

void rebuildGraph(uint8_t edgeA[], uint8_t edgeB[], int count) {
  memset(adjacency, 0, sizeof(adjacency));
  for (int i = 0; i < count; i++) {
    if (edgeA[i] > MAX_NODES || edgeB[i] > MAX_NODES) continue;
    adjacency[edgeA[i]][edgeB[i]] = 1;
    adjacency[edgeB[i]][edgeA[i]] = 1;
  }
}
void buildDefaultGraph() {
  memset(adjacency, 0, sizeof(adjacency));
  for (int i = 0; i < numDefaultEdges; i++) {
    adjacency[defaultEdges[i].a][defaultEdges[i].b] = 1;
    adjacency[defaultEdges[i].b][defaultEdges[i].a] = 1;
  }
}

// ---------- Peer state (must be declared before planners that use it) ----------
struct PeerHeartbeat { uint8_t current_node; uint8_t state; float eta_to_next; unsigned long last_seen; };
struct PeerRoute { uint8_t route[MAX_ROUTE_LEN]; uint8_t route_len; };
PeerHeartbeat peerHB[MAX_ROBOTS];
PeerRoute peerRoute[MAX_ROBOTS];
bool peer_known[MAX_ROBOTS] = {false,false,false,false,false,false};

const unsigned long PEER_TIMEOUT_MS = 6000;
bool isPeerStale(int r) { return (millis() - peerHB[r].last_seen) > PEER_TIMEOUT_MS; }

bool isNodeOccupied(uint8_t node) {
  for (int r = 1; r < MAX_ROBOTS; r++) {
    if (r == ROBOT_ID || !peer_known[r] || isPeerStale(r)) continue;
    if (peerHB[r].current_node == node) return true;
  }
  return false;
}

uint8_t getPeerNextNode(int r) {
  uint8_t curNode = peerHB[r].current_node;
  uint8_t len = peerRoute[r].route_len;
  for (int i = 0; i < len; i++) {
    if (peerRoute[r].route[i] == curNode) return (i+1 < len) ? peerRoute[r].route[i+1] : 255;
  }
  return 255;
}

bool isNodeNeededByAnother(uint8_t node, int exceptRobot) {
  for (int r = 1; r < MAX_ROBOTS; r++) {
    if (r == ROBOT_ID || !peer_known[r] || isPeerStale(r)) continue;
    if (r != exceptRobot && getPeerNextNode(r) == node) return true;
    uint8_t len = peerRoute[r].route_len;
    if (len > 0 && peerRoute[r].route[len-1] == node) return true; // final destination — never exclude the requester here
  }
  return false;
}

// ---------- Plain BFS (naive mode / fallback) ----------
bool bfs(uint8_t start, uint8_t target, uint8_t path[], uint8_t &pathLen) {
  bool visited[MAX_NODES+1]; memset(visited,0,sizeof(visited));
  uint8_t parent[MAX_NODES+1]; memset(parent,0,sizeof(parent));
  uint8_t queue[MAX_NODES+1]; int qHead=0,qTail=0;
  queue[qTail++]=start; visited[start]=true;
  while (qHead<qTail) {
    uint8_t cur = queue[qHead++];
    if (cur==target) break;
    for (int n=1;n<=MAX_NODES;n++) if (adjacency[cur][n] && !visited[n]) { visited[n]=true; parent[n]=cur; queue[qTail++]=n; }
  }
  if (!visited[target]) return false;
  uint8_t rev[MAX_NODES+1]; int len=0; uint8_t node=target;
  while(true){ rev[len++]=node; if(node==start) break; node=parent[node]; }
  for (int i=0;i<len;i++) path[i]=rev[len-1-i];
  pathLen=len;
  return true;
}

// ---------- SMART planner: Dijkstra with an "occupancy penalty" instead of
// a hard block, so the robot naturally avoids routing through wherever
// another robot (even an idle one) is currently sitting, but can still get
// there if that's genuinely the best/only option. ----------
const int OCCUPIED_PENALTY = 6;

bool dijkstraSmart(uint8_t start, uint8_t target, uint8_t path[], uint8_t &pathLen) {
  int dist[MAX_NODES+1];
  uint8_t parent[MAX_NODES+1];
  bool visited[MAX_NODES+1];
  for (int i=0;i<=MAX_NODES;i++) { dist[i]=INF; visited[i]=false; parent[i]=0; }
  dist[start]=0;

  for (int iter=0; iter<=MAX_NODES; iter++) {
    int u=-1, best=INF;
    for (int i=1;i<=MAX_NODES;i++) if (!visited[i] && dist[i]<best) { best=dist[i]; u=i; }
    if (u==-1) break;
    visited[u]=true;
    if (u==target) break;
    for (int v=1;v<=MAX_NODES;v++) {
      if (!adjacency[u][v]) continue;
      int cost = 1;
      if (isNodeOccupied((uint8_t)v)) cost += OCCUPIED_PENALTY;
      if (dist[u]+cost < dist[v]) { dist[v]=dist[u]+cost; parent[v]=u; }
    }
  }
  if (dist[target]>=INF) return false;
  uint8_t rev[MAX_NODES+1]; int len=0; uint8_t node=target;
  while(true){ rev[len++]=node; if(node==start) break; node=parent[node]; }
  for (int i=0;i<len;i++) path[i]=rev[len-1-i];
  pathLen=len;
  return true;
}

// Hard-exclude variant, used for reactive rerouting around a SPECIFIC
// contested node once already en route (different from the planning-time
// smart planner above, which only penalizes rather than blocks).
bool bfsAvoiding(uint8_t start, uint8_t target, uint8_t avoidNode, uint8_t path[], uint8_t &pathLen) {
  bool visited[MAX_NODES+1]; memset(visited,0,sizeof(visited));
  uint8_t parent[MAX_NODES+1]; memset(parent,0,sizeof(parent));
  uint8_t queue[MAX_NODES+1]; int qHead=0,qTail=0;
  if (avoidNode<=MAX_NODES) visited[avoidNode]=true;
  queue[qTail++]=start; visited[start]=true;
  while (qHead<qTail) {
    uint8_t cur=queue[qHead++];
    if (cur==target) break;
    for (int n=1;n<=MAX_NODES;n++) if (adjacency[cur][n] && !visited[n]) { visited[n]=true; parent[n]=cur; queue[qTail++]=n; }
  }
  if (!visited[target]) return false;
  uint8_t rev[MAX_NODES+1]; int len=0; uint8_t node=target;
  while(true){ rev[len++]=node; if(node==start) break; node=parent[node]; }
  for (int i=0;i<len;i++) path[i]=rev[len-1-i];
  pathLen=len;
  return true;
}

// ---------- TinyML ETA model (trained offline, see train_tiny_mlp.py) ----------
#define H 8
const float W1[2*H] = {0.050292f,-0.052842f,0.287045f,0.341791f,-0.214268f,0.257751f,0.350387f,0.409182f,-0.281494f,-0.506169f,-0.620676f,-0.170087f,-0.930012f,-0.864398f,-0.564021f,-0.455853f};
const float b1[H]   = {0.000000f,0.000000f,0.166552f,0.135517f,0.000000f,0.170846f,-0.199786f,0.173833f};
const float W2[H]   = {-0.217704f,-0.126520f,0.628202f,0.578702f,-0.051414f,1.054180f,-0.175518f,0.442703f};
const float b2      = 0.037455f;
const float DISTANCE_SCALE=30.0f, SPEED_SCALE=4.0f, TIME_SCALE=30.0f;

float predictETA(float distance, float speed) {
  float x1=distance/DISTANCE_SCALE, x2=speed/SPEED_SCALE;
  float hidden[H];
  for (int j=0;j<H;j++){ float z=x1*W1[0*H+j]+x2*W1[1*H+j]+b1[j]; hidden[j]=z>0?z:0; }
  float out=b2;
  for (int j=0;j<H;j++) out += hidden[j]*W2[j];
  float t=out*TIME_SCALE;
  return t>0?t:0;
}

// ---------- My state ----------
uint8_t my_route[MAX_ROUTE_LEN];
uint8_t my_route_len = 0;
uint8_t my_current_index = 0;
RobotState my_state = IDLE;
float my_speed = 3.5f;
float my_eta_to_next = 0;
bool smartMode = true;

uint16_t my_job_id = 0;
JobPhase my_job_phase = PHASE_NONE;
unsigned long my_job_start_time = 0;
unsigned long dwellUntil = 0;
unsigned long planSettleUntil = 0;

unsigned long travelStartTime = 0;
unsigned long lastRetryTime = 0;
unsigned long lastHeartbeat = 0;
unsigned long myWaitingSince = 0;
bool sidestepActive = false;
uint8_t sidestepRealDestination = 0;
const unsigned long RETRY_INTERVAL_MS = 400;
const unsigned long HEARTBEAT_INTERVAL_MS = 1500;
const unsigned long STARVATION_OVERRIDE_MS = 6000;
const unsigned long IDLE_YIELD_GRACE_MS = 3000;
const unsigned long PLAN_SETTLE_MS = 400;
const unsigned long PICKUP_DWELL_MS = 1500;

// ---------- Job table (shared/synced view, built from broadcasts) ----------
struct JobInfo {
  uint16_t job_id;
  uint8_t pickup_node;
  uint8_t dropoff_node;
  uint8_t target_robot;   // 0 = open to any idle robot; nonzero = reserved for that robot
  JobStatus status;
  uint8_t claimed_by;
  unsigned long claim_time;
  bool active;
};
JobInfo jobTable[MAX_JOBS];

int findJobSlot(uint16_t job_id) {
  for (int i=0;i<MAX_JOBS;i++) if (jobTable[i].active && jobTable[i].job_id==job_id) return i;
  return -1;
}
int findFreeOrEvictableSlot() {
  for (int i=0;i<MAX_JOBS;i++) if (!jobTable[i].active) return i;
  for (int i=0;i<MAX_JOBS;i++) if (jobTable[i].active && jobTable[i].status==JOB_DELIVERED) return i;
  return -1;
}

uint16_t combine16(uint8_t hi, uint8_t lo) { return ((uint16_t)hi << 8) | lo; }
void split16(uint16_t v, uint8_t &hi, uint8_t &lo) { hi = (v >> 8) & 0xFF; lo = v & 0xFF; }

// ---------- Wire messages ----------
struct HeartbeatMsg { uint8_t type; uint8_t robot_id; uint8_t current_node; uint8_t state; float eta_to_next; };
struct RouteMsg     { uint8_t type; uint8_t robot_id; uint8_t route[MAX_ROUTE_LEN]; uint8_t route_len; };
struct EventMsg     { uint8_t type; uint8_t robot_id; uint8_t event_type; uint8_t node; };
struct GraphConfigMsg { uint8_t type; uint8_t edge_count; uint8_t edges[MAX_EDGES][2]; };
struct ModeCommand  { uint8_t type; uint8_t target_robot_id; uint8_t smart_mode; };
struct JobAnnounceMsg { uint8_t type; uint8_t job_id_hi; uint8_t job_id_lo; uint8_t pickup_node; uint8_t dropoff_node; uint8_t target_robot; };
struct JobClaimMsg    { uint8_t type; uint8_t job_id_hi; uint8_t job_id_lo; uint8_t robot_id; };
struct JobCompleteMsg { uint8_t type; uint8_t job_id_hi; uint8_t job_id_lo; uint8_t robot_id; };
struct JobResultMsg   { uint8_t type; uint8_t job_id_hi; uint8_t job_id_lo; uint8_t robot_id; uint8_t mode; uint8_t dur_hi; uint8_t dur_lo; };

void sendHeartbeat() {
  HeartbeatMsg msg; msg.type=MSG_HEARTBEAT; msg.robot_id=ROBOT_ID;
  msg.current_node=my_route[my_current_index]; msg.state=my_state; msg.eta_to_next=my_eta_to_next;
  IPAddress b(255,255,255,255);
  udp.beginPacket(b,UDP_PORT); udp.write((uint8_t*)&msg,sizeof(msg)); udp.endPacket();
}
void sendRoute() {
  RouteMsg msg; msg.type=MSG_ROUTE; msg.robot_id=ROBOT_ID;
  memcpy(msg.route,my_route,sizeof(my_route)); msg.route_len=my_route_len;
  IPAddress b(255,255,255,255);
  for (int i=0;i<2;i++) { udp.beginPacket(b,UDP_PORT); udp.write((uint8_t*)&msg,sizeof(msg)); udp.endPacket(); }
}
void sendEvent(uint8_t eventType, uint8_t node) {
  EventMsg msg; msg.type=MSG_EVENT; msg.robot_id=ROBOT_ID; msg.event_type=eventType; msg.node=node;
  IPAddress b(255,255,255,255);
  udp.beginPacket(b,UDP_PORT); udp.write((uint8_t*)&msg,sizeof(msg)); udp.endPacket();
}
void sendJobAnnounce(uint16_t jid, uint8_t pickup, uint8_t dropoff, uint8_t targetRobot) {
  JobAnnounceMsg msg; msg.type=MSG_JOB_ANNOUNCE; split16(jid,msg.job_id_hi,msg.job_id_lo);
  msg.pickup_node=pickup; msg.dropoff_node=dropoff; msg.target_robot=targetRobot;
  IPAddress b(255,255,255,255);
  udp.beginPacket(b,UDP_PORT); udp.write((uint8_t*)&msg,sizeof(msg)); udp.endPacket();
}
void sendJobClaim(uint16_t jid) {
  JobClaimMsg msg; msg.type=MSG_JOB_CLAIM; split16(jid,msg.job_id_hi,msg.job_id_lo); msg.robot_id=ROBOT_ID;
  IPAddress b(255,255,255,255);
  udp.beginPacket(b,UDP_PORT); udp.write((uint8_t*)&msg,sizeof(msg)); udp.endPacket();
}
void sendJobComplete(uint16_t jid) {
  JobCompleteMsg msg; msg.type=MSG_JOB_COMPLETE; split16(jid,msg.job_id_hi,msg.job_id_lo); msg.robot_id=ROBOT_ID;
  IPAddress b(255,255,255,255);
  udp.beginPacket(b,UDP_PORT); udp.write((uint8_t*)&msg,sizeof(msg)); udp.endPacket();
}
void sendJobResult(uint16_t jid, unsigned long durationMs) {
  JobResultMsg msg; msg.type=MSG_JOB_RESULT; split16(jid,msg.job_id_hi,msg.job_id_lo);
  msg.robot_id=ROBOT_ID; msg.mode=smartMode?1:0;
  uint16_t deci=(uint16_t)min(65535UL, durationMs/100);
  split16(deci,msg.dur_hi,msg.dur_lo);
  IPAddress b(255,255,255,255);
  udp.beginPacket(b,UDP_PORT); udp.write((uint8_t*)&msg,sizeof(msg)); udp.endPacket();
}

// ---------- Job lifecycle ----------
void planRouteTo(uint8_t target);  // fwd decl
void abandonJob();

void handleJobClaim(uint16_t jid, uint8_t claimant) {
  int idx = findJobSlot(jid);
  if (idx == -1) {
    idx = findFreeOrEvictableSlot();
    if (idx == -1) return;
    jobTable[idx] = {jid, 0, 0, 0, JOB_CLAIMED, claimant, millis(), true};
  }
  JobInfo &j = jobTable[idx];
  if (j.status != JOB_CLAIMED || claimant < j.claimed_by) {
    j.status = JOB_CLAIMED;
    j.claimed_by = claimant;
    j.claim_time = millis();
  }
  if (my_job_id == jid && claimant != ROBOT_ID && claimant < ROBOT_ID) {
    Serial.println("Lost job claim race to a lower-ID robot — abandoning and returning to idle.");
    abandonJob();
  }
}

void abandonJob() {
  int idx = findJobSlot(my_job_id);
  if (idx != -1 && jobTable[idx].claimed_by == ROBOT_ID) {
    jobTable[idx].status = JOB_PENDING;
    jobTable[idx].claimed_by = 0;
  }
  my_job_id = 0;
  my_job_phase = PHASE_NONE;
  my_state = IDLE;
  my_route_len = 1;
  my_route[0] = my_route[my_current_index];
  my_current_index = 0;
  sendHeartbeat();
}

void onLegArrived() {
  if (my_job_phase == PHASE_TO_PICKUP) {
    sendEvent(EVT_ARRIVED_PICKUP, my_route[my_current_index]);
    my_job_phase = PHASE_DWELL_PICKUP;
    dwellUntil = millis() + PICKUP_DWELL_MS;
    my_state = IDLE;
    Serial.println("Arrived at pickup — loading...");
  } else if (my_job_phase == PHASE_TO_DROPOFF) {
    sendEvent(EVT_ARRIVED_DROPOFF, my_route[my_current_index]);
    unsigned long durationMs = millis() - my_job_start_time;
    sendJobComplete(my_job_id);
    sendJobResult(my_job_id, durationMs);
    int idx = findJobSlot(my_job_id);
    if (idx != -1) jobTable[idx].status = JOB_DELIVERED;
    Serial.print("Job "); Serial.print(my_job_id); Serial.print(" delivered in ");
    Serial.print(durationMs/1000.0); Serial.println("s");
    my_job_id = 0;
    my_job_phase = PHASE_NONE;
    my_state = IDLE;
  }
  sendHeartbeat();
}

void checkJobs() {
  if (my_job_id != 0) return;
  if (my_state != IDLE) return;

  // A job reserved specifically for me always takes priority over the
  // normal lowest-ID pool — this is what makes manual per-robot assignment
  // actually work rather than just being a suggestion.
  int reservedIdx = -1;
  for (int i = 0; i < MAX_JOBS; i++) {
    if (jobTable[i].active && jobTable[i].status == JOB_PENDING && jobTable[i].target_robot == ROBOT_ID) {
      if (reservedIdx == -1 || jobTable[i].job_id < jobTable[reservedIdx].job_id) reservedIdx = i;
    }
  }

  int bestIdx = reservedIdx;
  if (bestIdx == -1) {
    // No job reserved for me — fall back to the normal decentralized pool,
    // but skip anything reserved for a DIFFERENT robot; that job just
    // waits for its intended robot rather than being grabbed by whoever's
    // free first.
    for (int i = 0; i < MAX_JOBS; i++) {
      if (jobTable[i].active && jobTable[i].status == JOB_PENDING && jobTable[i].target_robot == 0) {
        if (bestIdx == -1 || jobTable[i].job_id < jobTable[bestIdx].job_id) bestIdx = i;
      }
    }
  }
  if (bestIdx == -1) return;

  uint16_t jid = jobTable[bestIdx].job_id;
  jobTable[bestIdx].status = JOB_CLAIMED;
  jobTable[bestIdx].claimed_by = ROBOT_ID;
  jobTable[bestIdx].claim_time = millis();

  my_job_id = jid;
  my_job_phase = PHASE_TO_PICKUP;
  my_job_start_time = millis();
  sendJobClaim(jid);
  sendEvent(EVT_JOB_CLAIMED, jobTable[bestIdx].pickup_node);
  Serial.print("Claimed job "); Serial.print(jid); Serial.print(": pickup ");
  Serial.print(jobTable[bestIdx].pickup_node); Serial.print(" -> dropoff "); Serial.println(jobTable[bestIdx].dropoff_node);
  planRouteTo(jobTable[bestIdx].pickup_node);
}

void planRouteTo(uint8_t target) {
  uint8_t startNode = my_route[my_current_index];
  uint8_t path[MAX_ROUTE_LEN]; uint8_t pathLen;
  bool found = smartMode ? dijkstraSmart(startNode, target, path, pathLen) : bfs(startNode, target, path, pathLen);
  if (!found) {
    Serial.println("No path found for this leg — abandoning job.");
    abandonJob();
    return;
  }
  memcpy(my_route, path, sizeof(path));
  my_route_len = pathLen;
  my_current_index = 0;
  myWaitingSince = 0;

  if (my_route_len <= 1) {
    onLegArrived();
    return;
  }

  my_state = PLANNING;
  planSettleUntil = millis() + PLAN_SETTLE_MS;
  sendRoute();
  sendHeartbeat();
  Serial.print("Planned route: ");
  for (int i=0;i<my_route_len;i++){ Serial.print(my_route[i]); if (i<my_route_len-1) Serial.print(" -> "); }
  Serial.println();
}

void checkIncoming() {
  uint8_t buf[250];
  int packetSize;
  while ((packetSize = udp.parsePacket()) > 0) {
    int len = udp.read(buf, sizeof(buf));
    if (len <= 0) continue;
    uint8_t type = buf[0];

    if (type == MSG_HEARTBEAT && len == sizeof(HeartbeatMsg)) {
      HeartbeatMsg msg; memcpy(&msg,buf,sizeof(msg));
      if (msg.robot_id != ROBOT_ID && msg.robot_id < MAX_ROBOTS) {
        peerHB[msg.robot_id] = {msg.current_node, msg.state, msg.eta_to_next, millis()};
        peer_known[msg.robot_id] = true;
      }
    } else if (type == MSG_ROUTE && len == sizeof(RouteMsg)) {
      RouteMsg msg; memcpy(&msg,buf,sizeof(msg));
      if (msg.robot_id != ROBOT_ID && msg.robot_id < MAX_ROBOTS) {
        memcpy(peerRoute[msg.robot_id].route, msg.route, sizeof(msg.route));
        peerRoute[msg.robot_id].route_len = msg.route_len;
      }
    } else if (type == MSG_GRAPH_CONFIG && len == sizeof(GraphConfigMsg)) {
      GraphConfigMsg cfg; memcpy(&cfg,buf,sizeof(cfg));
      uint8_t a[MAX_EDGES], bb[MAX_EDGES];
      for (int i=0;i<cfg.edge_count && i<MAX_EDGES;i++){ a[i]=cfg.edges[i][0]; bb[i]=cfg.edges[i][1]; }
      rebuildGraph(a, bb, cfg.edge_count);
      Serial.print("Graph config received: "); Serial.print(cfg.edge_count); Serial.println(" edges applied.");
    } else if (type == MSG_MODE_CMD && len == sizeof(ModeCommand)) {
      ModeCommand cmd; memcpy(&cmd,buf,sizeof(cmd));
      if (cmd.target_robot_id==ROBOT_ID || cmd.target_robot_id==0) {
        smartMode = (cmd.smart_mode==1);
        Serial.println(smartMode ? "Mode: SMART" : "Mode: NAIVE");
      }
    } else if (type == MSG_JOB_ANNOUNCE && len == sizeof(JobAnnounceMsg)) {
      JobAnnounceMsg msg; memcpy(&msg,buf,sizeof(msg));
      uint16_t jid = combine16(msg.job_id_hi, msg.job_id_lo);
      if (jid != 0 && findJobSlot(jid) == -1) {
        int idx = findFreeOrEvictableSlot();
        if (idx != -1) {
          jobTable[idx] = {jid, msg.pickup_node, msg.dropoff_node, msg.target_robot, JOB_PENDING, 0, 0, true};
          Serial.print("New job announced: "); Serial.print(jid);
          Serial.print(" ("); Serial.print(msg.pickup_node); Serial.print("->"); Serial.print(msg.dropoff_node);
          if (msg.target_robot != 0) { Serial.print(", reserved for Robot "); Serial.print(msg.target_robot); }
          Serial.println(")");
        }
      }
    } else if (type == MSG_JOB_CLAIM && len == sizeof(JobClaimMsg)) {
      JobClaimMsg msg; memcpy(&msg,buf,sizeof(msg));
      handleJobClaim(combine16(msg.job_id_hi, msg.job_id_lo), msg.robot_id);
    } else if (type == MSG_JOB_COMPLETE && len == sizeof(JobCompleteMsg)) {
      JobCompleteMsg msg; memcpy(&msg,buf,sizeof(msg));
      int idx = findJobSlot(combine16(msg.job_id_hi, msg.job_id_lo));
      if (idx != -1) jobTable[idx].status = JOB_DELIVERED;
    }
    // MSG_JOB_RESULT: only the dashboard needs this, robots ignore it.
  }
}

// A parked robot steps aside if a peer needs its spot — but never while
// it's actively holding a job (including its own pickup dwell).
void idleYieldCheck() {
  if (!smartMode) return;
  if (my_state != IDLE) return;
  if (my_job_id != 0) return;
  uint8_t myNode = my_route[my_current_index];

  for (int r = 1; r < MAX_ROBOTS; r++) {
    if (r == ROBOT_ID || !peer_known[r] || isPeerStale(r)) continue;
    if (peerHB[r].state == IDLE) continue;
    if (getPeerNextNode(r) != myNode) continue;

    for (int n = 1; n <= MAX_NODES; n++) {
      if (!adjacency[myNode][n]) continue;
      if (isNodeOccupied((uint8_t)n)) continue;
      if (isNodeNeededByAnother((uint8_t)n, r)) continue;

      my_route[0] = myNode; my_route[1] = (uint8_t)n; my_route_len = 2; my_current_index = 0;
      my_state = MOVING; my_eta_to_next = predictETA(1.0, my_speed); travelStartTime = millis();
      sendRoute(); sendHeartbeat(); sendEvent(EVT_YIELDED, (uint8_t)n);
      Serial.print("Yielding node "); Serial.print(myNode);
      Serial.print(" to Robot "); Serial.print(r);
      Serial.print(" — stepping aside to node "); Serial.println(n);
      return;
    }
    return;
  }
}

bool hasPriorityOver(int r) {
  if (peerHB[r].eta_to_next < my_eta_to_next) return false;
  if (peerHB[r].eta_to_next == my_eta_to_next && r < ROBOT_ID) return false;
  return true;
}

void attemptAdvance() {
  if (my_state != MOVING && my_state != WAITING) { Serial.println("Not currently moving."); return; }
  if (my_current_index + 1 >= my_route_len) {
    if (my_state != IDLE) { my_state = IDLE; sendHeartbeat(); }
    return;
  }

  uint8_t nextNode = my_route[my_current_index+1];
  uint8_t myCurrentNode = my_route[my_current_index];
  uint8_t destination = my_route[my_route_len-1];
  my_eta_to_next = predictETA(1.0, my_speed);

  // Who, if anyone, is CURRENTLY physically sitting on nextNode right now?
  int occupantId = -1;
  for (int r=1;r<MAX_ROBOTS;r++) {
    if (r==ROBOT_ID || !peer_known[r] || isPeerStale(r)) continue;
    if (peerHB[r].current_node == nextNode) { occupantId = r; break; }
  }

  if (occupantId != -1) {
    // ---- Rule 1: physical occupation is an UNCONDITIONAL block, no matter
    // who has priority. Priority never licenses walking into a node someone
    // is standing on — it only settles races for a node nobody occupies yet
    // (handled in Rule 2 below). This is the fix for a real collision bug
    // found via simulation: the old code let the higher-priority robot in a
    // head-on advance unconditionally, without ever confirming the target
    // was actually clear. ----
    bool occupantIdle = (peerHB[occupantId].state == IDLE);

    if (occupantIdle) {
      if (myWaitingSince==0) { myWaitingSince=millis(); sendEvent(EVT_WAITING, nextNode); }
      unsigned long waited = millis()-myWaitingSince;
      if (waited < IDLE_YIELD_GRACE_MS) {
        my_state=WAITING; sendHeartbeat();
        Serial.print("Node "); Serial.print(nextNode); Serial.print(" occupied by idle Robot ");
        Serial.print(occupantId); Serial.println(" — waiting briefly for it to clear.");
        return;
      }
      uint8_t altPath[MAX_ROUTE_LEN]; uint8_t altLen;
      if (bfsAvoiding(myCurrentNode, destination, nextNode, altPath, altLen)) {
        memcpy(my_route, altPath, sizeof(altPath)); my_route_len=altLen; my_current_index=0;
        my_eta_to_next=predictETA(1.0,my_speed); myWaitingSince=0; travelStartTime=millis();
        sendRoute(); sendHeartbeat(); sendEvent(EVT_REROUTED, nextNode);
        Serial.print("Robot "); Serial.print(occupantId); Serial.println(" isn't moving — rerouting around it instead.");
        return;
      }
      my_state=WAITING; sendHeartbeat();
      return;
    }

    // Occupant is live (moving/waiting/planning) — I must not enter,
    // regardless of priority. If it's a genuine head-on (it also wants my
    // current spot) and I'm the one who should yield, try to actually
    // clear space rather than just freezing in place.
    bool mutual = (getPeerNextNode(occupantId) == myCurrentNode);
    bool iHavePriority = hasPriorityOver(occupantId);

    if (mutual && !iHavePriority) {
      uint8_t altPath[MAX_ROUTE_LEN]; uint8_t altLen;
      if (bfsAvoiding(myCurrentNode, destination, nextNode, altPath, altLen)) {
        memcpy(my_route, altPath, sizeof(altPath)); my_route_len=altLen; my_current_index=0;
        my_eta_to_next=predictETA(1.0,my_speed); myWaitingSince=0; travelStartTime=millis();
        sendRoute(); sendHeartbeat(); sendEvent(EVT_REROUTED, nextNode);
        Serial.print("Head-on with Robot "); Serial.print(occupantId); Serial.println(" — rerouting around it.");
        return;
      }
      // No reroute possible — my own destination IS the contested node,
      // the classic unsolvable swap. Take a TEMPORARY sidestep to a third
      // node instead of freezing forever; that's what actually frees the
      // space for the other robot. Resume toward the real destination
      // once this sidestep leg completes (see the arrival handling below).
      for (int n=1;n<=MAX_NODES;n++) {
        if (n==nextNode || !adjacency[myCurrentNode][n]) continue;
        if (isNodeOccupied((uint8_t)n)) continue;
        if (isNodeNeededByAnother((uint8_t)n, occupantId)) continue;
        sidestepActive = true;
        sidestepRealDestination = destination;
        my_route[0]=myCurrentNode; my_route[1]=(uint8_t)n; my_route_len=2; my_current_index=0;
        my_eta_to_next=predictETA(1.0,my_speed); myWaitingSince=0; travelStartTime=millis();
        sendRoute(); sendHeartbeat(); sendEvent(EVT_YIELDED, (uint8_t)n);
        Serial.print("Head-on with Robot "); Serial.print(occupantId);
        Serial.print(" — no detour available, sidestepping to "); Serial.println(n);
        return;
      }
      // Truly boxed in with nowhere to sidestep — fall through to wait below.
    }

    // Not mutual, or I have priority, or no sidestep was available: I still
    // cannot enter an occupied node. Wait for it to actually clear.
    if (myWaitingSince==0) { myWaitingSince=millis(); sendEvent(EVT_WAITING, nextNode); }
    unsigned long waited = millis()-myWaitingSince;
    if (waited < STARVATION_OVERRIDE_MS) {
      my_state=WAITING; sendHeartbeat();
      Serial.print("WAITING before node "); Serial.print(nextNode);
      Serial.print(" - Robot "); Serial.print(occupantId); Serial.println(" is there.");
      return;
    }
    uint8_t altPath[MAX_ROUTE_LEN]; uint8_t altLen;
    if (bfsAvoiding(myCurrentNode, destination, nextNode, altPath, altLen)) {
      memcpy(my_route, altPath, sizeof(altPath)); my_route_len=altLen; my_current_index=0;
      my_eta_to_next=predictETA(1.0,my_speed); myWaitingSince=0; travelStartTime=millis();
      sendRoute(); sendHeartbeat(); sendEvent(EVT_REROUTED, nextNode);
      Serial.println("Starvation threshold hit — rerouting instead of forcing through.");
      return;
    }
    if (!isPeerStale(occupantId)) {
      my_state=WAITING; sendHeartbeat();
      Serial.println("Still blocked by a live peer with no alternate route — continuing to wait.");
      return;
    }
    Serial.println("Occupant has gone stale and no detour exists — proceeding.");
    // falls through to advance below

  } else {
    // ---- Rule 2: nextNode is currently EMPTY. Only here does priority
    // matter — resolving who gets to enter first when both are racing
    // toward a node neither occupies yet. ----
    int racerId = -1;
    for (int r=1;r<MAX_ROBOTS;r++) {
      if (r==ROBOT_ID || !peer_known[r] || isPeerStale(r)) continue;
      if (getPeerNextNode(r) == nextNode) { racerId = r; break; }
    }
    if (racerId != -1 && !hasPriorityOver(racerId)) {
      if (myWaitingSince==0) { myWaitingSince=millis(); sendEvent(EVT_WAITING, nextNode); }
      unsigned long waited = millis()-myWaitingSince;
      if (waited < STARVATION_OVERRIDE_MS) {
        my_state=WAITING; sendHeartbeat();
        Serial.print("WAITING before node "); Serial.print(nextNode);
        Serial.print(" - Robot "); Serial.print(racerId); Serial.println(" has priority.");
        return;
      }
      uint8_t altPath[MAX_ROUTE_LEN]; uint8_t altLen;
      if (bfsAvoiding(myCurrentNode, destination, nextNode, altPath, altLen)) {
        memcpy(my_route, altPath, sizeof(altPath)); my_route_len=altLen; my_current_index=0;
        my_eta_to_next=predictETA(1.0,my_speed); myWaitingSince=0; travelStartTime=millis();
        sendRoute(); sendHeartbeat(); sendEvent(EVT_REROUTED, nextNode);
        Serial.print("Starvation threshold hit (race with Robot "); Serial.print(racerId);
        Serial.println(") — rerouting instead of forcing through.");
        return;
      }
      if (!isPeerStale(racerId)) {
        my_state=WAITING; sendHeartbeat();
        Serial.print("Still racing live Robot "); Serial.print(racerId);
        Serial.println(" for that node with no alternate route — continuing to wait.");
        return;
      }
    }
  }

  bool wasWaiting = (myWaitingSince != 0);
  myWaitingSince = 0;
  my_current_index++;
  bool arrived = (my_current_index+1 >= my_route_len);
  my_state = arrived ? IDLE : MOVING;
  if (!arrived) my_eta_to_next = predictETA(1.0, my_speed);
  travelStartTime = millis();
  sendHeartbeat();

  if (arrived) {
    if (sidestepActive) {
      sidestepActive = false;
      Serial.print("Cleared sidestep — resuming toward "); Serial.println(sidestepRealDestination);
      planRouteTo(sidestepRealDestination);
    } else {
      onLegArrived();
    }
  } else if (wasWaiting) {
    sendEvent(EVT_RESUMED, my_route[my_current_index]);
  }

  Serial.print("Entered node "); Serial.println(my_route[my_current_index]);
}

void autoDrive() {
  unsigned long now = millis();
  idleYieldCheck();
  checkJobs();

  if (my_job_phase == PHASE_DWELL_PICKUP && now >= dwellUntil) {
    my_job_phase = PHASE_TO_DROPOFF;
    int idx = findJobSlot(my_job_id);
    uint8_t dropoff = (idx != -1) ? jobTable[idx].dropoff_node : my_route[my_current_index];
    planRouteTo(dropoff);
  }

  if (my_state == PLANNING) {
    if (now >= planSettleUntil) {
      my_state = MOVING;
      my_eta_to_next = predictETA(1.0, my_speed);
      travelStartTime = now;
      sendHeartbeat();
    }
  } else if (my_state == MOVING) {
    unsigned long travelMs = (unsigned long)(my_eta_to_next*1000);
    if (now - travelStartTime >= travelMs) attemptAdvance();
  } else if (my_state == WAITING) {
    if (now - lastRetryTime >= RETRY_INTERVAL_MS) { lastRetryTime = now; attemptAdvance(); }
  }

  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) { lastHeartbeat = now; sendHeartbeat(); }
}

char serialBuf[48];
uint8_t serialLen = 0;

void handleCommand(char* line) {
  if (strcmp(line, "n") == 0) {
    attemptAdvance();
  } else if (strcmp(line, "m") == 0) {
    smartMode = !smartMode;
    Serial.println(smartMode ? "Mode: SMART" : "Mode: NAIVE");
  } else if (line[0]=='j' && line[1]==' ') {
    int pickup=0, dropoff=0, targetRobot=0;
    int n = sscanf(line+2, "%d %d %d", &pickup, &dropoff, &targetRobot);
    if (n < 3) targetRobot = 0;  // "j 10 20" = open job; "j 10 20 2" = reserved for robot 2
    if (pickup>=1 && pickup<=MAX_NODES && dropoff>=1 && dropoff<=MAX_NODES) {
      uint16_t jid = (uint16_t)(esp_random() & 0xFFFF);
      if (jid==0) jid=1;
      int idx = findFreeOrEvictableSlot();
      if (idx!=-1) {
        jobTable[idx] = {jid,(uint8_t)pickup,(uint8_t)dropoff,(uint8_t)targetRobot,JOB_PENDING,0,0,true};
        sendJobAnnounce(jid,(uint8_t)pickup,(uint8_t)dropoff,(uint8_t)targetRobot);
        Serial.print("Created job "); Serial.print(jid); Serial.print(": ");
        Serial.print(pickup); Serial.print(" -> "); Serial.print(dropoff);
        if (targetRobot != 0) { Serial.print(" (reserved for Robot "); Serial.print(targetRobot); Serial.print(")"); }
        Serial.println();
      }
    } else {
      Serial.println("Usage: j <pickup> <dropoff> [robot_id]");
    }
  } else if (line[0]=='s' && line[1]==' ') {
    my_speed = atof(line+2);
    Serial.print("Speed set to "); Serial.println(my_speed);
  } else {
    int target = atoi(line);
    if (target>=1 && target<=MAX_NODES) {
      Serial.println("[debug move, bypasses job system]");
      planRouteTo((uint8_t)target);
    } else {
      Serial.println("Unknown command.");
    }
  }
}

void checkSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c=='\n' || c=='\r') {
      if (serialLen>0) { serialBuf[serialLen]='\0'; handleCommand(serialBuf); serialLen=0; }
    } else if (serialLen < sizeof(serialBuf)-1) {
      serialBuf[serialLen++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  buildDefaultGraph();
  for (int i=0;i<MAX_JOBS;i++) jobTable[i].active = false;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status()!=WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println();
  Serial.print("Connected. IP: "); Serial.println(WiFi.localIP());
  udp.begin(UDP_PORT);

  my_route[0]=1; my_route_len=1; my_current_index=0;

  Serial.print("Robot "); Serial.print(ROBOT_ID); Serial.println(" ready.");
  Serial.println("Commands: j <pickup> <dropoff> | m (toggle mode) | <node> (debug move) | n | s <speed>");
}

void loop() {
  checkIncoming();
  autoDrive();
  checkSerial();
}
