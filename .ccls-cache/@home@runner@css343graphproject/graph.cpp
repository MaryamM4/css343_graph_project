#include "graph.h"
#include <algorithm>
#include <climits>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <utility>
#include <vector>

using namespace std;

/**
 * Constructor, empty graph.
 * directionalEdges defaults to true.
 */
Graph::Graph(bool directionalEdges) {
  vertices[""] = new Vertex(""); // Dummy used for "invalid" values.
}

// Destructor.
Graph::~Graph() {
  for (const auto &vertex : vertices) {
    // Clear the vertex's 'edges'.
    for (std::pair<Vertex *, int> &edgePair : (vertex.second)->edges) {
      edgePair.first = nullptr;
    }
    (vertex.second)->edges.clear();

    // Deallocate the vertex pointers.
    vertices[vertex.first] = nullptr;
  }

  // Empty the 'vertices' map.
  vertices.clear();
}

// @return total number of vertices
int Graph::verticesSize() const { return vertices.size(); }

// @return total number of edges
int Graph::edgesSize() const { return 0; }

// @return number of edges from given vertex, -1 if vertex not found
int Graph::vertexDegree(const string &label) const { return -1; }

/**
 * Adds vertex to graph, without connecting to any edges.
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
 * @note   To check if output is "valid", use the isEdge(pair) function.
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
 * @note   Assumes the pair is an output
 *         of the getPair(src, edge) function.
 * @return true if pair is not equal to the dummy vertex ("is valid").
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

/**
 * Prints the Depth-first traversal starting from startLabel.
 */
void Graph::dfs(const string &startLabel,
                void visit(const string &startLabel)) {

  std::cout << "dfs(" << startLabel << ")\n";
  recDfs(startLabel, visit);
  resetVisits();
}

/**
 * Recusrive helper for displaying the DFS.
 *
 * Reference:
 * https://www.geeksforgeeks.org/depth-first-search-or-dfs-for-a-graph/
 */
void Graph::recDfs(const string &startLabel,
                   void visit(const string &startLabel)) {

  vertices[startLabel]->visited = true;
  visit(startLabel);

  for (std::pair<Vertex *, int> edgePair : (vertices[startLabel]->edges)) {
    if (!(edgePair.first->visited)) {
      recDfs(edgePair.first->data, visit);
    }
  }
}

/**
 * Prints the Breadth-first traversal starting from startLabel.
 *
 * Reference:
 * https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/#
 */
void Graph::bfs(const string &startLabel,
                void visit(const string &startLabel)) {

  if (!contains(startLabel)) {
    return;
  }

  queue<string> dataQ;

  vertices[startLabel]->visited = true;
  visit(startLabel);
  dataQ.push(startLabel);

  while (!dataQ.empty()) {
    string currVertexData = dataQ.front();
    dataQ.pop();

    vertices[currVertexData]->visited = false; // Reset

    for (std::pair<Vertex *, int> edgePair :
         (vertices[currVertexData]->edges)) {

      if (!(edgePair.first)->visited) {
        edgePair.first->visited = true;
        visit(edgePair.first->data);
        dataQ.push((edgePair.first)->data);
      }
    }
  }

  std::cout << std::endl;
  resetVisits();
}

// store the weights in a map
// store the previous label in a map

/**
 * Dijkstra's algorithm to find shortest distance to all other vertices
 * and the path to all other vertices.
 *
 * @return a pair made up of two map objects, Weights and Previous.
 *          Weights records the path cost; e.g. weight["F"] = 10
 *          Previous stores how to get to the it; e.g. previous["F"] = "C".
 *          Excludes vertices not connected to src.
 */
pair<map<string, int>, map<string, string>>
Graph::dijkstra(const string &srcLabel) const {
  map<string, int> weights;
  map<string, string> previous;

  // Fail case: src vertex DNE:
  if (!contains(srcLabel)) {
    std::cout << "Cannot call dijkstra's algorithm on non-existing vertex '"
              << srcLabel << "'." << std::endl;
    return make_pair(weights, previous);
  }

  vector<Vertex *> verts;

  for (const auto &entry : vertices) {
    if (entry.second->data != "") { // Ignore dummy.
      weights[entry.first] = INT_MAX;
      previous[entry.first] = "";

      entry.second->visited = false; // Make sure it's reset.
      verts.push_back(entry.second);
    }
  }

  weights[srcLabel] = 0;
  previous[srcLabel] = srcLabel;

  while (!verts.empty()) {
    Vertex *closestVertex = verts[0];
    int minCost = INT_MAX;

    // Find the closest vertex
    for (Vertex *vert : verts) {
      if (weights[vert->data] <= minCost) {
        closestVertex = vert;
        minCost = weights[closestVertex->data];
      }
    }

    // Remove closest vertex from list of remaining
    verts.erase(std::remove(verts.begin(), verts.end(), closestVertex),
                verts.end());
    closestVertex->visited = true;

    // Iterate through the neighbors of the closest vertex
    for (const auto &edgePair : closestVertex->edges) {
      // + weight from clostVertex to it's current edge
      int altMinCost = weights[closestVertex->data] + edgePair.second;

      if (!(edgePair.first->visited) &&
          altMinCost <= weights[edgePair.first->data]) {
        weights[edgePair.first->data] = altMinCost;
        previous[edgePair.first->data] = closestVertex->data;
      }
    }

    // Deal with vertices not connected to src:
    if (std::abs(weights[closestVertex->data]) >= INT_MAX) {
      // Exclude:
      // weights.erase(closestVertex->data);
      // previous.erase(closestVertex->data);

      // Imply:
      weights[closestVertex->data] = INT_MAX;
      previous[closestVertex->data] = "";
    }
  }

  resetVisits();
  return make_pair(weights, previous);
}

/**
 * Minimum spanning tree using Prim's algorithm.
 *
 * @note ONLY works for NONDIRECTED graphs, and
 *       ASSUMES the edge [P->Q] has the same weight as [Q->P].
 *
 * @return length of the minimum spanning tree,
 *          or -1 if start vertex DNE.
 */
int Graph::mstPrim(const string &startLabel,
                   void visit(const string &from, const string &to,
                              int weight)) const {

  if (!contains(startLabel)) {
    return -1;
  }

  // TODO

  resetVisits();
  return -1;
}

/**
 * Minimum spanning tree using Kruskal's algorithm.
 *
 * @note ONLY works for NONDIRECTED graphs, and
 *       ASSUMES the edge [P->Q] has the same weight as [Q->P].
 *
 * @return length of the minimum spanning tree,
 *          or -1 if start vertex DNE.
 */
int Graph::mstKruskal(const string &startLabel,
                      void visit(const string &from, const string &to,
                                 int weight)) const {

  if (!contains(startLabel)) {
    return -1;
  }

  // TODO

  resetVisits();
  return -1;
}

/**
 * Read edges from a text file and creates the graph.
 *
 * First line of file is an integer, indicating number of edges.
 * Each line represents an edge in the form of "string string int"
 * (vertex labels cannot contain spaces).
 *
 * @pre    File must exist and follow specified format.
 *         Filename should end with extension ".txt".
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

/**
 *  Calls dijkstra function and prints outcome.
 */
void Graph::printDijkstra(const string &startLabel) {
  std::cout << "\ndijkstra(" << startLabel << "):\n";

  // Weights = dij.first, Prev = dij.second
  pair<map<string, int>, map<string, string>> dij = dijkstra(startLabel);

  for (const auto &elem : dij.first) {
    std::cout << "" << elem.first << ": weight = ";

    if (std::abs(elem.second) == INT_MAX) {
      std::cout << "INF.\n";

    } else {
      std::cout << elem.second << ", prev = " << dij.second[elem.first]
                << ".\n";
    }
  }

  std::cout << std::endl;
}

/**
 * Visit helper function for traversal methods.
 */
void Graph::visit(const string &label) {
  if (contains(label)) {
    vertices[label]->visited = true;
    // std::cout << "Visited vertex '" << label << "'.\n";

  } else {
    std::cout << "Cannot visit non-existent vertex '" << label << "'.\n";
  }
}

/**
 * Resets all vertices' "visit" bool to false.
 */
void Graph::resetVisits() const {
  for (const auto &entry : vertices) {
    (entry.second)->visited = false;
  }
  std::cout << "\nAll vertices 'visit' status reset to false.\n\n";
}