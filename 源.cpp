#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>

using namespace std;

// 图数据结构
class Graph {
private:
    int V; // 顶点数
    vector<vector<int>> adj; // 邻接表

public:
    Graph(int V) {
        this->V = V;
        adj.resize(V);
    }

    void addEdge(int u, int v) {
        adj[u].push_back(v);
        adj[v].push_back(u);
    }

    // BFS算法
    void BFS(int s) {
        vector<bool> visited(V, false);
        queue<int> q;

        visited[s] = true;
        q.push(s);

        while (!q.empty()) {
            int u = q.front();
            q.pop();
            cout << u << " ";

            for (int v : adj[u]) {
                if (!visited[v]) {
                    visited[v] = true;
                    q.push(v);
                }
            }
        }
    }

    // DFS算法
    void DFSUtil(int u, vector<bool>& visited) {
        visited[u] = true;
        cout << u << " ";

        for (int v : adj[u]) {
            if (!visited[v]) {
                DFSUtil(v, visited);
            }
        }
    }

    void DFS(int s) {
        vector<bool> visited(V, false);
        DFSUtil(s, visited);
    }

    // 最短路径算法
    void shortestPath(int s) {
        vector<int> dist(V, INT_MAX);
        vector<bool> visited(V, false);
        dist[s] = 0;

        for (int i = 0; i < V - 1; i++) {
            int u = -1;
            for (int j = 0; j < V; j++) {
                if (!visited[j] && (u == -1 || dist[j] < dist[u])) {
                    u = j;
                }
            }

            visited[u] = true;
            for (int v : adj[u]) {
                if (!visited[v]) {
                    dist[v] = min(dist[v], dist[u] + 1);
                }
            }
        }

        for (int i = 0; i < V; i++) {
            cout << "Shortest distance from " << s << " to " << i << " is " << dist[i] << endl;
        }
    }

    // 最小支撑树算法
    void primMST() {
        vector<int> parent(V, -1);
        vector<int> key(V, INT_MAX);
        vector<bool> visited(V, false);
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

        int src = 0;
        key[src] = 0;
        pq.push(make_pair(0, src));

        while (!pq.empty()) {
            int u = pq.top().second;
            pq.pop();

            visited[u] = true;
            for (int v : adj[u]) {
                if (!visited[v] && key[v] > 1) {
                    key[v] = 1;
                    pq.push(make_pair(key[v], v));
                    parent[v] = u;
                }
            }
        }

        for (int i = 1; i < V; i++) {
            cout << parent[i] << " - " << i << endl;
        }
    }
};

int main() {
    Graph g(6);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 3);
    g.addEdge(2, 4);
    g.addEdge(3, 4);
    g.addEdge(3, 5);
    g.addEdge(4, 5);

    cout << "BFS traversal starting from vertex 0: ";
    g.BFS(0);
    cout << endl;

    cout << "DFS traversal starting from vertex 0: ";
    g.DFS(0);
    cout << endl;

    cout << "Shortest path from vertex 0: " << endl;
    g.shortestPath(0);

    cout << "Minimum spanning tree: " << endl;
    g.primMST();

    return 0;
}