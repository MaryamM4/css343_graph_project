#include "graph.h"
#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

using namespace std;

// constructor, empty graph
// directionalEdges defaults to true
Graph::Graph(bool directionalEdges) {
  // Dummy used for "invalid" values.
  vertices[""] = new Vertex("");
}

// destructor
Graph::~Graph() {}

// @return total number of vertices
int Graph::verticesSize() const { return 0; }

// @return total number of edges
int Graph::edgesSize() const { return 0; }

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const { return -1; }

/**
 * @return true if vertex added, false if it already is in the graph.
 */
bool Graph::add(const string &label) {
  if (label == "" || contains(label)) {
    return false;
  }

  vertices[label] = new Vertex(label);
  return true;
}

/**
 * @return true if vertex is already in 'vertices' map,
 *         false otherwise.
 */
bool Graph::contains(const string &label) const {
  if (label == "") {
    return false;
  }

  return (vertices.find(label) != vertices.end());
}

/**
 * @returns pair from vertexes edges
 *          pair<Vertex *edge, int: directCost>.
 *          On invalid case, returns the end iterator:
 *          vertices[src]->edges.end().
 *
 * Invalid case 1: 'src' or 'to' don't already exist in 'vertices' map.
 * Invalid case 2: 'to' is not an edge of 'src'.
 *
 * @note   To check if output is valid, use the isEdge(pair) function.
 */
vector<std::pair<Vertex *, int>>::iterator
Graph::getPair(const std::string &src, const std::string &edgeVal) {
  if (contains(src) && contains(edgeVal)) {
    std::vector<std::pair<Vertex *, int>> &srcEdges = vertices[src]->edges;

    for (auto it = srcEdges.begin(); it != srcEdges.end(); ++it) {
      if (it->first->data == edgeVal) {
        return it; // Return iterator to the pair
      }
    }
  }

  return vertices[""]->edges.end(); // Return end iterator if not found
}

/**
 * @note   Assumes the pair is an output of the getPair(src, edge) function.
 * @return true if pair is valid.
 */
bool Graph::isEdge(vector<std::pair<Vertex *, int>>::iterator pair) {
  return (pair != vertices[""]->edges.end());
}

/** return -1 if edge is not directly pointed to by src. */
int Graph::costToEdge(const string &src, const string edgeVal) {
  vector<std::pair<Vertex *, int>>::iterator pair = getPair(src, edgeVal);

  if (pair != vertices[src]->edges.end()) {
    return pair->second;
  }

  return -1;
}

//
/**
 * @return string representing edges and weights, without newlines,
 *         or return "" if vertex not found.
 *
 * Ex: Input  = "A",
 *     where     A-3->B, A-5->C
 *     Output = "B(3), C(5)"
 */
string Graph::getEdgesAsString(const string &srcLabel) const {
  string edges = "";

  for (const auto &edgePair : vertices.at(srcLabel)->edges) {
    if (!edges.empty()) {
      edges += ", ";
    }

    edges += edgePair.first->data + "(" + to_string(edgePair.second) + ")";
  }

  return edges;
}

// @return true if successfully connected
bool Graph::connect(const string &src, const string &to, int weight) {
  vector<std::pair<Vertex *, int>>::iterator pair = getPair(src, to);

  // Return false if already connected.
  if (isEdge(pair)) {
    return false;
  }

  // At this point, 'src' and 'to' both already exist as vertices in map
  vertices[src]->edges.push_back(std::make_pair(vertices[to], weight));
  return true;
}

/*
 * Removes 'to' from 'src's edges, but not vice-versa.
 * @return false if not already connected, and true if disconnected.
 */
bool Graph::disconnect(const string &src, const string &to) {
  vector<std::pair<Vertex *, int>>::iterator pair = getPair(src, to);

  if (isEdge(pair)) {
    vertices[src]->edges.erase(pair);
    return true;
  }

  // Return false if not already connected.
  return false;
}

// depth-first traversal starting from given startLabel
void Graph::dfs(const string &startLabel, void visit(const string &label)) {}

// breadth-first traversal starting from startLabel
void Graph::bfs(const string &startLabel, void visit(const string &label)) {}

// store the weights in a map
// store the previous label in a map
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &startLabel) const {
  map<string, int> weights;
  map<string, string> previous;
  // TODO(student) Your code here
  return make_pair(weights, previous);
}

// minimum spanning tree using Prim's algorithm
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {
  return -1;
}

// minimum spanning tree using Prim's algorithm
int Graph::mstKruskal(const string &startLabel,
                      void visit(const string &from, const string &to,
                                 int weight)) const {
  return -1;
}

/**
 * Read edges from a text file and creates the graph.
 *
 * First line of file is an integer, indicating number of edges.
 * Each line represents an edge in the form of "string string int"
 * (vertex labels cannot contain spaces).
 *
 * @return true if file successfully read, false otherwise.
 */
bool Graph::buildFromFile(const string &filename) {
  ifstream myfile(filename);

  if (!myfile.is_open()) {
    cerr << "Failed to open " << filename << endl;
    return false;
  }

  int edges = 0;
  int weight = 0;
  string fromVertex;
  string toVertex;

  myfile >> edges; // First line in file represents num of edges.
  for (int i = 0; i < edges; ++i) {
    myfile >> fromVertex >> toVertex >> weight;

    // Create new Vertex objects if they DNE.
    add(fromVertex);
    add(toVertex);

    connect(fromVertex, toVertex, weight);
  }

  myfile.close();
  return true;
}

/**
 * Prints values in 'verticies' map, and vertex's 'edges' vector.
 */
void Graph::printVertexEdges() {
  for (const auto &vertex : vertices) {
    if (vertex.first != "") {

      std::cout << "[" << vertex.first << "] " << getEdgesAsString(vertex.first)
                << std::endl;
    }
  }

  std::cout << std::endl;
}
