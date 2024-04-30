/**
 * Driver for tests
 */

#include "graph.h"
#include <iostream>

using namespace std;

string startLabel = "A";

/*********
 * VISITS
 **********/

void visit(const string &vertexLabel) {
  cout << "Visited vertex: " << vertexLabel << endl;
}

void visit(const string &from, const string &to, int weight) {
  cout << from << "--" << to << " " << weight << std::endl;
}

/*********
 * TESTS
 **********/

void fileBuild(Graph &graph, string filename) {
  Graph gA;
  graph.buildFromFile(filename);
  cout << endl;
  gA.printVertexEdges();
}

void testDij(Graph &graph) {
  cout << endl;
  graph.printDijkstra(startLabel);
}

void testBFS(Graph &graph) {
  cout << "\nbfs(" << startLabel << ")\n";
  graph.bfs(startLabel, visit);
}

void testDFS(Graph &graph) {
  // cout << "\ndfs(" << startLabel << ")\n";
  graph.dfs(startLabel, visit);
}

// forward declaration, implementation in xxxtest.cpp
void testAll();

/*********
 * MAIN
 **********/
int main() {
  Graph gOne;
  fileBuild(gOne, "graph1.txt");

  testDij(gOne);
  testBFS(gOne);
  testDFS(gOne);

  // testAll();

  cout << "\nDone!" << endl;
  return 0;
}
