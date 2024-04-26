/**
 * Driver for tests
 */

#include "graph.h"
#include <iostream>

using namespace std;

void testFileBuild() {
  Graph gA;
  gA.buildFromFile("graph1.txt");
  cout << endl;
  gA.printVertexEdges();
}

// forward declaration, implementation in xxxtest.cpp
void testAll();

int main() {
  testFileBuild();

  // testAll();

  cout << "Done!" << endl;
  return 0;
}
