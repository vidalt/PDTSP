#include "pdp2optmove.h"

#include "pdp/pdproute.h"
#include "utils/application.h"

using namespace std;
namespace pdp {
namespace moves {

PDP2optMove::PDP2optMove() {
  visited = new bool[Application::instance->Size()];
}

PDP2optMove::~PDP2optMove() {
  delete[] visited;
}

PDPMoveEvaluation PDP2optMove::Evaluate(PDPSolution *solution, PDPNode *pickupNode) {
  PDPNode **nodes = static_cast<PDPNode **>(Application::instance->Data());
  double **distances = Application::instance->Distances();

  PDPMoveEvaluation eval;
  eval.cost = DBL_MAX;
  eval.neighborhood = this;
  eval.moveparam = &state;

  int positionPickup = solution->FindPosition(pickupNode->idx);
  int positionDelivery = solution->FindPosition(pickupNode->pair);
  PDPRoute &route = solution->route;

  memset(visited, 0, Application::instance->Size() * sizeof(bool));
  visited[pickupNode->idx] = true;

  double bestCostDelta = DBL_MAX;
  int bestEnd = 0;
  int bestStart = positionPickup;
  for (int i = positionPickup + 1; i < positionDelivery; i++) {
    PDPNode *node = nodes[route[i]];

    // if its closing a P-D then break... 2opt is now invalid.
    if (node->isDelivery && visited[node->pair]) break;

    visited[node->idx] = true;

    double costDelta = -distances[route[positionPickup - 1]][route[positionPickup]] -
                       distances[route[i]][route[i + 1]] + distances[route[positionPickup - 1]][route[i]] +
                       distances[route[positionPickup]][route[i + 1]];

    if (costDelta < bestCostDelta) {
      bestCostDelta = costDelta;
      bestEnd = i;
    }
  }

  memset(visited, 0, Application::instance->Size() * sizeof(bool));
  for (size_t i = positionDelivery + 1; i < route.size() - 1; i++) {
    PDPNode *node = nodes[route[i]];

    if (node->isDelivery && visited[node->pair]) break;

    visited[node->idx] = true;

    double costDelta = -distances[route[positionDelivery - 1]][route[positionDelivery]] -
                       distances[route[i]][route[i + 1]] + distances[route[positionDelivery - 1]][route[i]] +
                       distances[route[positionDelivery]][route[i + 1]];

    if (costDelta < bestCostDelta) {
      bestStart = positionDelivery;
      bestEnd = i;
      bestCostDelta = costDelta;
    }
  }

  if (bestCostDelta < DBL_MAX) {
    state.originPickup = positionPickup;
    state.originDelivery = positionDelivery;
    state.destinyPickup = bestStart;
    state.destinyDelivery = bestEnd;
    eval.cost = solution->cost + bestCostDelta;
  }

  return eval;
}

PDPMoveEvaluation PDP2optMove::Evaluate(PDPSolution * /*solution*/) {
  PDPMoveEvaluation eval;
  eval.cost = DBL_MAX;
  eval.neighborhood = this;
  eval.moveparam = &state;

  return eval;
}

double PDP2optMove::move(PDPSolution *solution, const PDPMoveEvaluation &eval) {
  PDP2optMoveState *curState = (PDP2optMoveState *)eval.moveparam;

  PDPRoute *route = &solution->route;

  int idxStart = curState->destinyPickup;
  int idxEnd = curState->destinyDelivery;

  std::reverse(route->begin() + idxStart, route->begin() + idxEnd + 1);

  solution->cost -= route->Cost();
  solution->cost += route->PrecomputeRouteInformation();

  return PDPMove::move(solution, eval);
}

std::string PDP2optMove::name() const {
  return "pdp_2-opt";
}

}  // namespace moves
}  // namespace pdp
