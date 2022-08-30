#include "instancereader.h"

#include <float.h>

#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/tuple/tuple.hpp>
#include <fstream>
#include <iostream>

#include "pdp/instancereader.h"
#include "pdpnode.h"
#include "utils/application.h"

using namespace std;

namespace pdp {

/// RBO00 FORMAT
InstanceReader::~InstanceReader() {
}

bool InstanceReader::understands(const string /*instanceFilePath*/) {
  return true;
}

PDPInstance* InstanceReader::fromFile(const string instanceFilePath) {
  // cout << "Reading instance " << instanceFilePath << endl;

  ifstream in(instanceFilePath, ifstream::in);
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

  // DEPOT
  NodeList nodelist;
  do {
    std::getline(ss, myline);
    boost::algorithm::trim(myline);
  } while (myline.empty() && !ss.eof());
  PDPNode* pNode = new PDPNode();
  liness = stringstream(myline);
  liness >> pNode->idx;
  liness >> pNode->x;
  liness >> pNode->y;
  pNode->idx = 0;  // depot
  pNode->pair = 0;
  pNode->isPickup = false;    // is pickup
  pNode->isDelivery = false;  // is delivery
  nodelist.push_back(pNode);
  ////////////////////////////////

  int idxCount = 1;
  while (!ss.eof()) {
    std::getline(ss, myline);
    boost::algorithm::trim(myline);
    if (!myline.size()) continue;
    liness = stringstream(myline);
    int idx;
    liness >> idx;

    if (idx == -999) break;

    pNode = new PDPNode();
    pNode->idx = idxCount++;  // customer index
    liness >> pNode->x;       // x coord
    liness >> pNode->y;       // y coord

    int flag;
    liness >> flag;
    liness >> pNode->pair;
    pNode->pair--;

    pNode->isDelivery = flag;
    pNode->isPickup = !pNode->isDelivery;

    nodelist.push_back(pNode);
  }
  return new PDPInstance(std::max(Application::hgsadc_cl, 1), nodelist);
}

// GRUBHUB FORMAT
InstanceReaderGrubhub::~InstanceReaderGrubhub() {
}

bool InstanceReaderGrubhub::understands(const string /*instanceFilePath*/) {
  return true;
}

PDPInstance* InstanceReaderGrubhub::fromFile(const string instanceFilePath) {
  ifstream in(instanceFilePath, ifstream::in);
  string content((std::istreambuf_iterator<char>(in)), (std::istreambuf_iterator<char>()));
  stringstream ss(content);

  string myline;
  stringstream liness;
  std::vector<std::string> splittedtoks;
  std::getline(ss, myline);
  boost::algorithm::trim(myline);  // Instance Name
  std::getline(ss, myline);
  boost::algorithm::trim(myline);  // Number of nodes
  boost::algorithm::split(splittedtoks, myline, boost::algorithm::is_any_of(":"));
  boost::algorithm::trim(splittedtoks[1]);
  liness = stringstream(splittedtoks[1]);

  int numberOfNodes;
  liness >> numberOfNodes;

  NodeList nodelist;
  PDPNode* node;
  for (int i = 0; i < numberOfNodes; i++) {
    node = new PDPNode();
    node->idx = i;
    node->x = node->y = 0;
    node->pair = 0;
    node->isPickup = false;    // is pickup
    node->isDelivery = false;  // is delivery
    if (i > 0) {
      node->pair = (i % 2) == 1 ? i + 1 : i - 1;
      node->isPickup = (i % 2) == 1;       // is pickup
      node->isDelivery = !node->isPickup;  // is delivery
    }
    nodelist.push_back(node);
  }

  PDPInstance* instance = new PDPInstance(std::max(Application::hgsadc_cl, 1), nodelist, "");

  double**& distances = Application::instance->Distances();
  distances = new double*[numberOfNodes];

  for (int i = 0; i < numberOfNodes; i++) {
    int dist;
    distances[i] = new double[numberOfNodes];

    std::getline(ss, myline);
    boost::algorithm::trim(myline);
    liness = stringstream(myline);

    for (int j = 0; j < numberOfNodes; j++) {
      liness >> dist;
      distances[i][j] = (i == j) ? 0.0 : int(dist + 0.5);
    }
  }

  return instance;
}

}  // namespace pdp
