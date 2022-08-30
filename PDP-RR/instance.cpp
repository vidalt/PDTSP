#include "instance.h"

#include <float.h>
#include <math.h>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/tuple/tuple.hpp>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

bool Solution::operator<(const Solution& sol) {
  return cost < sol.cost;
}

Solution::Solution() {
  cost = 0.0;
}

Instance::Instance(const string instanceFilePath) {
  distances = nullptr;
  // cout << "Reading instance " << instanceFilePath << endl;

  ifstream in(instanceFilePath, ifstream::in);

  if (!in.is_open()) {
    throw std::invalid_argument("Unable to read instance file.");
  }

  string content((std::istreambuf_iterator<char>(in)), (std::istreambuf_iterator<char>()));
  stringstream ss(content);

  // RBO00
  string myline;
  stringstream liness;
  std::getline(ss, myline);
  boost::algorithm::trim(myline);  // Number of nodes
  liness = stringstream(myline);

  int nnodes;
  liness >> nnodes;

  ////////////////DEPOT//////////
  do {
    std::getline(ss, myline);
    boost::algorithm::trim(myline);
  } while (myline.empty() && !ss.eof());

  Node* node = new Node();
  liness = stringstream(myline);
  liness >> node->idx;
  liness >> node->x;
  liness >> node->y;
  node->idx = 0;  // depot
  node->pair = 0;
  node->isPickup = false;    // is pickup
  node->isDelivery = false;  // is delivery
  nodes.push_back(node);
  ////////////////////////////////

  int flag;
  int idxCount = 1;
  while (!ss.eof()) {
    std::getline(ss, myline);
    boost::algorithm::trim(myline);
    if (!myline.size()) continue;
    liness = stringstream(myline);
    int idx;
    liness >> idx;

    if (idx == -999) break;

    node = new Node();
    node->idx = idxCount++;  // customer index
    liness >> node->x;       // x coord
    liness >> node->y;       // y coord

    liness >> flag;
    liness >> node->pair;
    node->pair--;

    node->isDelivery = flag;
    node->isPickup = !node->isDelivery;

    nodes.push_back(node);
  }

  CreateDistanceMatrix();
}

void Instance::CreateDistanceMatrix() {
  size_t numberOfNodes = nodes.size();
  distances = new double*[numberOfNodes];

  for (size_t i = 0; i < numberOfNodes; i++) {
    distances[i] = new double[numberOfNodes];
  }

  for (size_t i = 0; i < numberOfNodes; i++) {
    distances[i][i] = 0.0;

    double dx, dy;
    for (size_t j = i + 1; j < numberOfNodes; j++) {
      dx = nodes[i]->x - nodes[j]->x;
      dy = nodes[i]->y - nodes[j]->y;
      distances[i][j] = distances[j][i] = int(sqrt(dx * dx + dy * dy) + 0.5);
    }
  }
}

Instance::NodeList Instance::Pickups() const {
  NodeList pickupNodes;
  for (const Node* node : nodes) {
    if (node->isPickup) pickupNodes.push_back(node);
  }
  return pickupNodes;
}

Instance::~Instance() {
  if (distances != nullptr) {
    for (size_t i = 0; i < nodes.size(); i++) {
      delete[] distances[i];
    }
    delete[] distances;
  }

  NodeList::iterator it;
  for (it = nodes.begin(); it != nodes.end(); it++) {
    delete *it;
  }
  nodes.clear();
}
