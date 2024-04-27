/**
 * Driver for tests
 */

#include "graph.h"
#include <iostream>

using namespace std;

void testFileBuild(const string &filename) {
  Graph gA;
  gA.buildFromFile(filename);
  cout << endl;
  gA.printVertexEdges();
}

void testDij(const string &filename) {
  Graph gA;
  gA.buildFromFile(filename);
  cout << endl;
  gA.printDijkstra("A");
}

// forward declaration, implementation in xxxtest.cpp
void testAll();

int main() {
  string filename = "graph1.txt";

  // testFileBuild(filename);
  testDij(filename);

  // testAll();

  cout << "Done!" << endl;
  return 0;
}
