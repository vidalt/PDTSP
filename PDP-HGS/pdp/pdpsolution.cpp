#include "pdpsolution.h"

#include <iostream>

#include "pdp/pdpinstance.h"
#include "pdp/pdproute.h"
#include "utils/application.h"

namespace pdp {

using namespace std;

PDPSolution::PDPSolution() {
  idx = -1;
}

PDPSolution::PDPSolution(const PDPSolution& s) {
  *this = s;
}

const PDPSolution& PDPSolution::operator=(const PDPSolution& s) {
  cost = s.cost;
  positions = s.positions;

  route = s.route;

  return *this;
}

PDPSolution::~PDPSolution() {
}

ga::Solution* PDPSolution::Clone() const {
  return new PDPSolution(*this);
}

void PDPSolution::Copy(const ga::Solution* s) {
  *this = *(const PDPSolution*)s;
}

bool PDPSolution::IsFeasible() const {
  PDPNode** nodes = static_cast<PDPNode**>(Application::instance->Data());

  vector<bool> visited;
  visited.resize(Application::instance->Size(), false);
  visited[0] = true;

  for (size_t i = 1; i < route.size() - 1; i++) {
    PDPNode* node = nodes[route[i]];
    if (visited[node->idx] || (node->isDelivery && !visited[node->pair])) {
      return false;
    }
    visited[node->idx] = true;
  }

  return true;
}

void PDPSolution::Print(ostream& os) const {
  if (Application::verbose) {
    os << "\tCOST: " << this->Cost() << endl;
    os << "\tROUTE: ";
  } else {
    os << "  \"solution\": ";
  }
  route.Print(os);
}

double PDPSolution::Cost() const {
  return cost;
}

void PDPSolution::Recompute() {
  ComputePositions();
  cost = route.PrecomputeRouteInformation();
}

void PDPSolution::ComputePositions() {
  positions.resize(Application::instance->Size());
  std::fill(positions.begin(), positions.end(), -1);

  for (size_t i = 1; i < route.size() - 1; i++) {
    positions[route[i]] = i;
  }
}

int PDPSolution::FindPosition(int k) const {
  if (k >= positions.size()) return -1;
  return positions[k];
}

}  // namespace pdp
