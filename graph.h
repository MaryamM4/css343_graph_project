/**
 * A graph is made up of vertices and edges.
 * Vertex labels are unique.
 * A vertex can be connected to other vertices via weighted, directed edge.
 * A vertex cannot connect to itself or have multiple edges to the same vertex
 */

#ifndef GRAPH_H
#define GRAPH_H

#include <map>
#include <queue>
#include <string>
#include <vector>

using namespace std;

// Graph node.
struct Vertex {
  Vertex(string val) : data(val), visited(false) {}

  // EX:  A-[3]->B  & A-[8]->C :
  //      data  = "A"
  //      edges = [{B*, 3}, {C*, 8}]
  string data;
  vector<pair<Vertex *, int>> edges;

  mutable bool visited; // Whether vertex has been visited.
};

// Union for Krustal's algorithm. Adjusted from:
// https://www.geeksforgeeks.org/kruskals-minimum-spanning-tree-algorithm-greedy-algo-2/
class DisjointSet {
public:
  DisjointSet() {}

  void makeSet(Vertex *vertex) {
    parent[vertex] = vertex;
    rank[vertex] = 0;
  }

  Vertex *findSet(Vertex *vertex) {
    if (vertex != parent[vertex]) {
      parent[vertex] = findSet(parent[vertex]);
    }
    return parent[vertex];
  }

  void unionSets(Vertex *x, Vertex *y) {
    Vertex *rootX = findSet(x);
    Vertex *rootY = findSet(y);

    if (rank[rootX] < rank[rootY]) {
      parent[rootX] = rootY;

    } else if (rank[rootX] > rank[rootY]) {
      parent[rootY] = rootX;

    } else {
      parent[rootY] = rootX;
      rank[rootX]++;
    }
  }

private:
  map<Vertex *, Vertex *> parent;
  map<Vertex *, int> rank;
};

class Graph {
public:
  // constructor, empty graph
  explicit Graph(bool directionalEdges = true);

  // copy not allowed
  Graph(const Graph &other) = delete;

  // move not allowed
  Graph(Graph &&other) = delete;

  // assignment not allowed
  Graph &operator=(const Graph &other) = delete;

  // move assignment not allowed
  Graph &operator=(Graph &&other) = delete;

  /** destructor, delete all vertices and edges */
  ~Graph();

  // @return true if vertex added, false if it already is in the graph
  bool add(const string &label);

  // @return true if vertex is in the graph
  bool contains(const string &label) const;

  // @returns end iterator if not found (vertices[src]->edges.end())
  vector<std::pair<Vertex *, int>>::iterator
  getPair(const std::string &src, const std::string &edgeVal);

  // @reutrn true if the pair is a valid pair.
  bool isEdge(vector<std::pair<Vertex *, int>>::iterator pair);

  // @returns cost to direct edge.
  //           -1 if edge is not directly pointed at by src.
  int costToEdge(const string &src, const string edge);

  // @return total number of vertices
  int verticesSize() const;

  // Add an edge between two vertices, create new vertices if necessary
  // A vertex cannot connect to itself, cannot have P->P
  // For digraphs (directed graphs), only one directed edge allowed, P->Q
  // Undirected graphs must have P->Q and Q->P with same weight
  // @return true if successfully connected
  bool connect(const string &from, const string &to, int weight = 0);

  // Remove edge from graph
  // @return true if edge successfully deleted
  bool disconnect(const string &from, const string &to);

  // @return total number of edges
  int edgesSize() const;

  // @return number of edges from given vertex, -1 if vertex not found
  int vertexDegree(const string &label) const;

  // @return string representing edges and weights, "" if vertex not found
  // A-3->B, A-5->C should return B(3),C(5)
  string getEdgesAsString(const string &label) const;

  // Read edges from file
  // first line of file is an integer, indicating number of edges
  // each line represents an edge in the form of "string string int"
  // vertex labels cannot contain spaces
  // @return true if file successfully read
  // Input should end with ".txt"
  bool buildFromFile(const string &filename);

  // Sole purpose of this function is to adhere to grader's tests.
  bool readFile(const string &filename) { return buildFromFile(filename); }

  // depth-first traversal starting from given startLabel
  void dfs(const string &startLabel, void visit(const string &label));
  void recDfs(const string &startLabel, void visit(const string &label));

  // breadth-first traversal starting from startLabel
  // call the function visit on each vertex label */
  void bfs(const string &startLabel, void visit(const string &label));

  // dijkstra's algorithm to find shortest distance to all other vertices
  // and the path to all other vertices
  // Path cost is recorded in the map passed in, e.g. weight["F"] = 10
  // How to get to the vertex is recorded previous["F"] = "C"
  // @return a pair made up of two map objects, Weights and Previous
  pair<map<string, int>, map<string, string>>
  dijkstra(const string &startLabel) const;

  // minimum spanning tree using Prim's algorithm
  // ONLY works for NONDIRECTED graphs
  // ASSUMES the edge [P->Q] has the same weight as [Q->P]
  // @return length of the minimum spanning tree or -1 if start vertex not
  int mstPrim(const string &startLabel,
              void visit(const string &from, const string &to,
                         int weight)) const;

  // minimum spanning tree using Kruskal's algorithm
  // ONLY works for NONDIRECTED graphs
  // ASSUMES the edge [P->Q] has the same weight as [Q->P]
  // @return length of the minimum spanning tree or -1 if start vertex not
  int mstKruskal(const string &startLabel,
                 void visit(const string &from, const string &to,
                            int weight)) const;

  void visit(const string &label);

  // IF start and edge are direct neighbors, return cost between.
  int costToEdge(const string &startLabel, const string &edgeLabel);

  std::pair<Vertex *, int> closestUnvisitedEdge(const string &startLabel) const;

  int minKey(int vSize, int key[], bool mst[]);

  bool ascendingEdgeSort(const std::pair<Vertex *, int> &leftEdge,
                         const std::pair<Vertex *, int> &rightEdge);

  // priority_queue<pair<int, pair<Vertex *, Vertex *>>> getSortedEdges();

  multimap<int, pair<Vertex *, Vertex *>> getSortedEdges() const;

  // ====================================
  // ============== PRINTS ==============

  // Prints values in 'verticies' map, and vertex's 'edges' vector.
  void printVertexEdges();

  // Calls dijkstra function and prints outcome.
  void printDijkstra(const string &startLabel);

private:
  // "vertex" -> [{"adjVertex1*, costTo1"}, ...]
  map<std::string, Vertex *> vertices;
  void resetVisits() const;
};

#endif // GRAPH_H