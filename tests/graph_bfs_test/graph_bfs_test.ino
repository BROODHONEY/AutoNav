// Standalone graph + BFS shortest path test — no networking involved.
// Type two numbers separated by a space in Serial Monitor, e.g. "2 3"
// (with line ending set to Newline) to compute the shortest route
// from node 2 to node 3.

#include <Arduino.h>

#define MAX_NODES 20

struct Edge { uint8_t a; uint8_t b; };

// Define your warehouse graph here — edit this to match your actual layout.
Edge edges[] = {
  {1,2}, {1,10}, {2,3}, {3,4}, {4,5}, {5,6}, {6,7}, {7,8}, {3,8}, {8,9},
  {9,10}, {10,11}, {11,12}, {12,13}, {13,14}, {8,13}, {14,15}, {15,16},
  {16,17}, {17,18}, {15,18}, {18,19}, {19,20}, {11,20}
};
const int numEdges = sizeof(edges) / sizeof(edges[0]);

uint8_t adjacency[MAX_NODES + 1][MAX_NODES + 1]; // adjacency[a][b] = 1 if connected
uint8_t parent[MAX_NODES + 1];
bool visited[MAX_NODES + 1];

void buildGraph() {
  memset(adjacency, 0, sizeof(adjacency));
  for (int i = 0; i < numEdges; i++) {
    adjacency[edges[i].a][edges[i].b] = 1;
    adjacency[edges[i].b][edges[i].a] = 1;
  }
}

// Returns true if a path was found, fills 'path' array and 'pathLen'
bool bfs(uint8_t start, uint8_t target, uint8_t path[], int &pathLen) {
  memset(visited, 0, sizeof(visited));
  memset(parent, 0, sizeof(parent));

  uint8_t queue[MAX_NODES + 1];
  int qHead = 0, qTail = 0;

  queue[qTail++] = start;
  visited[start] = true;
  parent[start] = 0;

  while (qHead < qTail) {
    uint8_t current = queue[qHead++];
    if (current == target) break;

    for (int n = 1; n <= MAX_NODES; n++) {
      if (adjacency[current][n] && !visited[n]) {
        visited[n] = true;
        parent[n] = current;
        queue[qTail++] = n;
      }
    }
  }

  if (!visited[target]) return false;

  // reconstruct path backwards
  uint8_t reversed[MAX_NODES + 1];
  int len = 0;
  uint8_t node = target;
  while (node != 0) {
    reversed[len++] = node;
    if (node == start) break;
    node = parent[node];
  }

  // reverse into forward order
  for (int i = 0; i < len; i++) {
    path[i] = reversed[len - 1 - i];
  }
  pathLen = len;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  buildGraph();
  Serial.println("Graph ready. Type: <start> <target>  e.g. 2 3");
}

void loop() {
  if (Serial.available()) {
    int start = Serial.parseInt();
    int target = Serial.parseInt();
    while (Serial.available()) Serial.read(); // clear trailing newline

    if (start < 1 || start > MAX_NODES || target < 1 || target > MAX_NODES) {
      Serial.println("Invalid node numbers.");
      return;
    }

    uint8_t path[MAX_NODES];
    int pathLen = 0;

    if (bfs(start, target, path, pathLen)) {
      Serial.print("Route from ");
      Serial.print(start);
      Serial.print(" to ");
      Serial.print(target);
      Serial.print(": ");
      for (int i = 0; i < pathLen; i++) {
        Serial.print(path[i]);
        if (i < pathLen - 1) Serial.print(" -> ");
      }
      Serial.println();
    } else {
      Serial.println("No path found.");
    }
  }
}
