#include "pdpinstance.h"

#include <math.h>

#include <algorithm>
#include <iostream>

#include "instancereader.h"
#include "moves/pdp2koptmove.h"
#include "moves/pdp2optmove.h"
#include "moves/pdpbsmove.h"
#include "moves/pdpmoveevaluation.h"
#include "moves/pdporoptmove.h"
#include "moves/pdprelocatemove.h"
#include "pdproute.h"
#include "utils/application.h"
#include "utils/random.h"

using namespace std;

namespace pdp {
void PDPInstance::PrecomputeClosest(int /*closeindividuals*/) {
  closest.resize(numberOfNodes);

  // FILL ALL CUSTOMERS IDS
  std::vector<int> nodeids;
  nodeids.resize(numberOfNodes);
  for (size_t i = 0; i < numberOfNodes; i++) {
    nodeids[i] = i;
  }
  ///////////////////////////////

  fartherClose.resize(numberOfNodes);

  for (size_t i = 0; i < numberOfNodes; i++) {
    // sort by distance ASC
    std::sort(nodeids.begin(), nodeids.end(), [this, i](const int customerA, const int customerB) {
      double d1 = (customerA != i) ? distances[i][customerA] : DBL_MAX;
      double d2 = (customerB != i) ? distances[i][customerB] : DBL_MAX;

      return d1 < d2;
    });

    closest[i] = nodeids;
    fartherClose[i] = distances[i][nodeids[nclosest]];
  }
}

void PDPInstance::PrecomputeDistanceMatrix() {
  if (!distances) {
    distances = new double*[numberOfNodes];

    for (size_t i = 0; i < numberOfNodes; i++) {
      distances[i] = new double[numberOfNodes];
    }

    double dx;
    double dy;
    for (size_t i = 0; i < numberOfNodes; i++) {
      distances[i][i] = 0.0;
      for (size_t j = i + 1; j < numberOfNodes; j++) {
        dx = nodes[i]->x - nodes[j]->x;
        dy = nodes[i]->y - nodes[j]->y;

        double dist = (double)sqrt(dx * dx + dy * dy);
        distances[i][j] = distances[j][i] = (int)(dist + 0.5);
      }
    }
  }
}

PDPInstance* PDPInstance::fromFilePath(const string instanceFilePath) {
  PDPInstance* instance = nullptr;
  InstanceReader* instanceReader = Application::grubhubmode
                                       ? static_cast<InstanceReader*>(new InstanceReaderGrubhub())
                                       : static_cast<InstanceReader*>(new InstanceReader());

  if (instanceReader->understands(instanceFilePath)) instance = instanceReader->fromFile(instanceFilePath);

  delete instanceReader;
  return instance;
}

PDPInstance::PDPInstance(int numberOfCloseIndividuals, const NodeList& nodes, const std::string comment)
    : vehicles(1),
      numberOfNodes(nodes.size()),
      nclosest(numberOfCloseIndividuals),
      comment(comment),
      nodes(nodes) {
  _visited = nullptr;
  _sucessor = nullptr;

  pEducate = nullptr;
  pRelocateMove = nullptr;
  p4optMove = nullptr;

  distances = nullptr;
}

PDPInstance::~PDPInstance() {
  if (_visited) delete[] _visited;
  if (_sucessor) delete[] _sucessor;
  if (pEducate) delete pEducate;
  if (pRelocateMove) delete pRelocateMove;
  if (p4optMove) delete p4optMove;

  if (distances != nullptr) {
    for (size_t i = 0; i < numberOfNodes; i++) {
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

void PDPInstance::Precompute() {
  PrecomputeDistanceMatrix();
  PrecomputeClosest(nclosest);

  if (pEducate) delete pEducate;

  pEducate = new pdp::Educate();
  p4optMove = new pdp::moves::PDP4optMove();
  pRelocateMove = new pdp::moves::PDPRelocateMove();

  if (Application::ls_relocate) pEducate->push_back(new pdp::moves::PDPRelocateMove());

  if (Application::ls_2opt) pEducate->push_back(new pdp::moves::PDP2optMove());

  if (Application::ls_2kopt) pEducate->push_back(new pdp::moves::PDP2koptMove());

  if (Application::ls_4opt_cd || Application::ls_4opt_dc || Application::ls_4opt_dd)
    pEducate->push_back(new pdp::moves::PDP4optMove());

  if (Application::ls_oropt) pEducate->push_back(new pdp::moves::PDPOroptMove());

  if (Application::ls_bs) pEducate->push_back(new pdp::moves::PDPBsMove());

  _visited = new bool[Application::instance->Size() + 2];
  _sucessor = new int[Application::instance->Size()];
}

int PDPInstance::CreateRandomSolution(PDPSolution* solution) const {
  PDPNode** nodes = static_cast<PDPNode**>(Application::instance->Data());

  vector<PDPNode*> pickupNodes;
  int szNodes = numberOfNodes;
  for (int i = 1; i < szNodes; i++) {
    if (nodes[i]->isPickup) pickupNodes.push_back(nodes[i]);
  }

  solution->route.clear();
  solution->route.push_back(0);
  solution->route.push_back(0);
  solution->ComputePositions();
  solution->cost = 0.0;

  Random::shuffle(pickupNodes.begin(), pickupNodes.end());
  for (PDPNode* node : pickupNodes) {
    pdp::moves::PDPMoveEvaluation evaluation = pRelocateMove->Evaluate(solution, node);
    evaluation.Apply(solution, true);
  }

  solution->Recompute();
  return 0;
}

ga::Solution* PDPInstance::CreateEmptySolution() const {
  return new PDPSolution();
}

ga::Solution* PDPInstance::CreateRandomSolution() const {
  PDPSolution* s = (PDPSolution*)CreateEmptySolution();
  CreateRandomSolution(s);
  return s;
}

void PDPInstance::Educate(ga::Solution* _s) {
  PDPSolution* s = (PDPSolution*)_s;
  pEducate->Run(s);
  s->Recompute();
  Sort(s);
}

size_t PDPInstance::Size() const {
  return numberOfNodes;
}

void* PDPInstance::Data() {
  return nodes.data();
}

bool compareNodes(int n1, int n2) {
  PDPNode** instance = static_cast<PDPNode**>(Application::instance->Data());

  PDPNode* N1 = instance[n1];
  PDPNode* N2 = instance[n2];

  if (N1->isPickup == N2->isPickup) return n1 < n2;

  return N1->isPickup;
}

void PDPInstance::Sort(PDPSolution* solution) {
  PDPRoute* route = &solution->route;
  for (size_t i = 1; i < route->size(); i++) {
    for (size_t j = 1; j < route->size() - i - 1; j++) {
      double costDelta = -distances[(*route)[j - 1]][(*route)[j]] - distances[(*route)[j]][(*route)[j + 1]] -
                         distances[(*route)[j + 1]][(*route)[j + 2]] +
                         distances[(*route)[j - 1]][(*route)[j + 1]] +
                         distances[(*route)[j + 1]][(*route)[j]] + distances[(*route)[j]][(*route)[j + 2]];

      if ((costDelta == 0) && compareNodes((*route)[j + 1], (*route)[j])) {
        std::swap((*route)[j], (*route)[j + 1]);
      }
    }
  }

  solution->Recompute();
}

void PDPInstance::Mutate(ga::Solution* _solution) {
  PDPSolution* solution = (PDPSolution*)_solution;
  pdp::moves::PDPMoveEvaluation moveA;
  moveA = ((pdp::moves::PDP4optMove*)p4optMove)->Evaluate(solution, true);
  moveA.Apply(solution, true);
}

std::string PDPInstance::LSCompleteLog() {
  return pEducate->TotalMovesLog();
}

std::string PDPInstance::LSLog() {
  return pEducate->MovesLog();
}

void PDPInstance::LSLogReset() {
  pEducate->ResetCounts();
}

void PDPInstance::Crossover(ga::Solution* _child, const ga::Solution* _p1, const ga::Solution* _p2) {
  PDPSolution* child = (PDPSolution*)_child;
  const PDPSolution* p1 = (const PDPSolution*)_p1;
  const PDPSolution* p2 = (const PDPSolution*)_p2;

  const std::vector<int>* _r1 = &p1->route;
  const std::vector<int>* _r2 = &p2->route;
  // randomly selected child
  if (Random::RandomReal() <= 0.5) {
    _r2 = &p1->route;
    _r1 = &p2->route;
  }

  const std::vector<int>& r1 = *_r1;
  const std::vector<int>& r2 = *_r2;

  const int n = (int)r1.size() - 2;

  std::vector<int>& newgt = child->route;
  newgt.clear();
  newgt.resize(r1.size(), 0);

  std::vector<int> contained;
  contained.resize((unsigned long)n + 1, 0);  //+1 to fit node 0 - depot.

  // Cutting points
  int s = Random::RandomInt(0, n);
  int e;

  do {
    e = Random::RandomInt(0, n);
  } while (e == s);

  if (s > e) std::swap(s, e);

  for (int p = s; p <= e; p++)  // copy part of gt1
  {
    newgt[p + 1] = r1[p + 1];
    contained[r1[p + 1]] = 1;
  }

  int k = 0;
  for (int i = 0; i < n; i++) {
    if (!contained[r2[i + 1]]) {
      while (k >= s && k <= e)
        k++;

      newgt[k + 1] = r2[i + 1];
      contained[r2[i + 1]] = 1;
      k++;
    }
  }

  child->Recompute();
}

void PDPInstance::Repair(ga::Solution* _solution) {
  PDPSolution* solution = (PDPSolution*)_solution;
  PDPNode** instance = static_cast<PDPNode**>(Application::instance->Data());

  vector<PDPNode*> nodes;
  vector<int> nodesIdx;
  vector<double> nodesCost;

  PDPRoute& route = solution->route;
  int n = route.size();
  int p;

  for (p = 0; p < n; p++) {
    PDPNode* node = instance[route[p]];
    if (node->isPickup && solution->FindPosition(node->pair) < p) {
      nodes.push_back(node);
      nodesIdx.push_back(nodesIdx.size());
    }
  }

  Random::shuffle(nodesIdx.begin(), nodesIdx.end());
  for (size_t i = 0; i < nodes.size(); i++) {
    pdp::moves::PDPMoveEvaluation evaluation = pRelocateMove->Evaluate(solution, nodes[nodesIdx[i]]);
    evaluation.Apply(solution, true);
  }

  solution->Recompute();
}

double PDPInstance::SolutionDistance(const ga::Solution* _a, const ga::Solution* _b) const {
  const PDPSolution* a = (const PDPSolution*)_a;
  const PDPSolution* b = (const PDPSolution*)_b;

  memset(_sucessor, 0, sizeof(int) * numberOfNodes);
  _sucessor[0] = a->route[1];
  for (size_t i = 0; i < a->route.size() - 1; i++)
    _sucessor[a->route[i]] = a->route[i + 1];

  size_t lenintersection = 0;
  for (size_t i = 0; i < b->route.size() - 1; i++) {
    if (_sucessor[b->route[i]] == b->route[i + 1]) {
      lenintersection++;
    }
  }

  size_t lenunion = 2 * (a->route.size() - 1) - lenintersection;
  return static_cast<double>(lenunion - lenintersection) / static_cast<double>(lenunion);
}

}  // namespace pdp
