#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <cmath> // Include for the heuristic function

using namespace std;

#define INF numeric_limits<int>::max()

// Structure to represent a node and its weight in the graph
struct Edge {
    int dest;
    int w;
};

// Define the heuristic function
int heuristic(int current, int goal) {
    // Replace this with an appropriate heuristic for your problem
    return abs(goal - current);
}

// Function to perform A* search algorithm
void astar(vector<vector<Edge>>& graph, int sn, int en) {
    int nn = graph.size();
    vector<int> dist(nn, INF); // Initialize distances with infinity
    vector<int> p(nn, -1); // Initialize parent nodes as -1

    dist[sn] = 0; // Distance to the start node is 0
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push(make_pair(heuristic(sn, en), sn)); // Push the start node to the priority queue with the heuristic estimate

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (const Edge& neighbor : graph[u]) {
            int v = neighbor.dest;
            int weight = neighbor.w;

            // Relaxation using the combined cost and heuristic estimate
            if (dist[u] != INF && dist[v] > dist[u] + weight + heuristic(v, en)) {
                dist[v] = dist[u] + weight;
                p[v] = u;
                pq.push(make_pair(dist[v] + heuristic(v, en), v)); // Update the priority queue with the combined cost and heuristic estimate
            }
        }
    }

    // Output shortest distance and path
    cout << "Shortest distance from Node " << sn << " to Node " << en << ": " << dist[en] << endl;
    cout << "Shortest path: ";
    int cn = en;
    while (cn != -1) {
        cout << cn << " ";
        cn = p[cn];
    }
    cout << endl;
}

int main() {
    // Example graph representation using adjacency list
    int numNodes = 6;
    vector<vector<Edge>> graph(numNodes);

    // Adding edges and weights to the graph
    // (Same as before)

    int sn, en;
    cout << "Enter the starting node: ";
    cin >> sn;
    cout << "Enter the ending node: ";
    cin >> en;

    astar(graph, sn, en);

    return 0;
}