Graph

//----------------1. Bellman ford algorithm-----------------

Time - O(V*E), Space - O(V)

for (int i = 0; i < V; i++)
    dist[i] = INT_MAX;
dist[src] = 0;

// Step 2: Relax all edges |V| - 1 times. A simple shortest
// path from src to any other vertex can have at-most |V| - 1
// edges
for (int i = 1; i <= V - 1; i++) {
    for (int j = 0; j < E; j++) {
        int u = graph->edge[j].src;
        int v = graph->edge[j].dest;
        int weight = graph->edge[j].weight;
        if (dist[u] != INT_MAX && dist[u] + weight < dist[v])
            dist[v] = dist[u] + weight;
    }
}

// Step 3: check for negative-weight cycles.  The above step
// guarantees shortest distances if graph doesn't contain
// negative weight cycle.  If we get a shorter path, then there
// is a cycle.
for (int i = 0; i < E; i++) {
    int u = graph->edge[i].src;
    int v = graph->edge[i].dest;
    int weight = graph->edge[i].weight;
    if (dist[u] != INT_MAX && dist[u] + weight < dist[v]) {
        printf("Graph contains negative weight cycle");
        return; // If negative cycle is detected, simply return
    }
}

//---------------2. Dijkstra's algorithm-------------
O(E log(V))

void dijkstra()
{
    set<pair<int, int>> st;
    memset(dist, INT_MAX, sizeof dist);
    dist[src] = 0;
    st.insert(0, src);
    while (!st.empty())
    {
        pair<int, int>curr = *st.begin();
        st.erase(st.begin());
        int u = curr.second;
        int currDist = curr.first;
        for (auto next : v[u])
        {
            int v = next.first;
            int w = next.second;
            if (dist[u] + w < dist[v])
            {
                auto it = st.find({dist[v], v});
                if (it != it.end()) {
                    st.erase(f);
                }
                dist[v] = dist[u] + w;
                st.insert(make_pair(dist[v], v));
            }
        }
    }
}

Why Prim’s and Kruskal’s MST algorithm fails for Directed Graph ?


//-------------------3. Kahn's algorithm for topological sorting------------------
(Indegree == 0)

// The initial state of the algorithm requires the addition of
// indegree = 0 nodes */
    for (int i = 0; i < indegree.length; i++) {
        if (indegree[i] == 0)
            queue.add(i);
    }
// Loop through the entire list until the indegrees of all nodes
// become zero
while (!queue.isEmpty()) {

    int current = queue.poll();
    sortedList.add(current);
    List<Integer> neighbors = graph.get(current);
    for (Integer i : neighbors) {
        indegree.get(i)--;
        if (indegree.get(i) == 0) {
            queue.add(i);
        }
    }


//-----------------   4. Kruskal's algorithm-----------------
    // (E log(V))

    // Sort the graph edges with respect to their weights.
    // Start adding edges to the MST from the edge with the smallest weight until the edge of the largest weight.
    // Only add edges which doesn't form a cycle , edges which connect only disconnected components.

    long long kruskal(pair<long long, pair<int, int> > p[])
    {
        int x, y;
        long long cost, minimumCost = 0;
        for (int i = 0; i < edges; ++i)
        {
            // Selecting edges one by one in increasing order from the beginning
            x = p[i].second.first;
            y = p[i].second.second;
            cost = p[i].first;
            // Check if the selected edge is creating a cycle or not
            if (root(x) != root(y))
            {
                minimumCost += cost;
                union1(x, y);
            }
        }
        return minimumCost;
    }


//---------------   5. Prim's algorithm---------------
    (E log(V))

    long long prim(int x)
    {
        priority_queue<PII, vector<PII>, greater<PII> > Q;
        int y;
        long long minimumCost = 0;
        PII p;
        Q.push(make_pair(0, x));
        while (!Q.empty())
        {
            // Select the edge with minimum weight
            p = Q.top();
            Q.pop();
            x = p.second;
            // Checking for cycle
            if (marked[x] == true)
                continue;
            minimumCost += p.first;
            marked[x] = true;
            for (int i = 0; i < adj[x].size(); ++i)
            {
                y = adj[x][i].second;
                if (marked[y] == false)
                    Q.push(adj[x][i]);
            }
        }
        return minimumCost;
    }

//-------------------Tarjan's Algorithm for SCC in a directed graph------------------

    void dfs(int u, vector<int>& disc, vector<int>& low, stack<int>& mystack, vector<bool>& presentInStack) {
        static int time = -1;
        time++;
        disc[u] = low[u] = time;
        mystack.push(u);
        presentInStack[u] = true;
        for (auto v : graph[u]) {
            if (disc[v] == -1) {
                dfs(u, disc, low, mystack, presentInStack);
                low[u] = min(low[u], low[v]);
            }
            else {
                low[u] = min(low[u], disc[v]);
            }
        }
        if (low[u] == dis[u]) {
            while (mystack.top() != u) {
                cout << mystack.top();
                presentInStack[mystack.top()] = false;
                mystack.pop();
            }
            cout << mystack.top();
            presentInStack[mystack.top()] = false;
            mystack.pop();
        }
    }

    void findSCCs_Tarjan()
    {
        vector<int> disc(V, -1), low(V, -1);
        vector<bool> presentInStack(V, false);  //Avoids cross-edge
        stack<int> mystack;

        for (int i = 0; i < V; ++i)
            if (disc[i] == -1)
                dfs(i, disc, low, mystack, presentInStack);
    }

