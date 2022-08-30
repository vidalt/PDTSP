#include "pdprelocatemove.h"

#include "pdp/pdproute.h"
#include "utils/application.h"
#include "utils/random.h"

using namespace std;
using namespace ga;

namespace pdp {
namespace moves {

PDPRelocateMove::PDPRelocateMove() {
  routeToWork = new int[Application::instance->Size() + 2];
}

PDPRelocateMove::~PDPRelocateMove() {
  delete[] routeToWork;
}

PDPMoveEvaluation PDPRelocateMove::Evaluate(PDPSolution* /*solution*/) {
  PDPMoveEvaluation eval;
  eval.cost = DBL_MAX;
  eval.neighborhood = this;
  eval.moveparam = &state;

  return eval;
}

PDPMoveEvaluation PDPRelocateMove::Evaluate(PDPSolution* solution, PDPNode* pickupNode) {
  double** distances = Application::instance->Distances();

  PDPMoveEvaluation eval;
  eval.cost = DBL_MAX;
  eval.neighborhood = this;
  eval.moveparam = &state;
  double removingDelta = 0.0;
  int routeSZ = 0;
  int positionPickup = solution->FindPosition(pickupNode->idx);
  int positionDelivery = solution->FindPosition(pickupNode->pair);

  // calculate delta to remove pickup and delivery from original route
  PDPRoute& originalRoute = solution->route;

  // Preparing Working Route
  PDPRoute& inseringWork = solution->route;
  for (size_t i = 0; i < inseringWork.size(); i++) {
    if (inseringWork[i] == pickupNode->idx || inseringWork[i] == pickupNode->pair) continue;

    routeToWork[routeSZ++] = inseringWork[i];
  }

  if (positionPickup != -1) {
    if (std::abs(positionDelivery - positionPickup) > 1) {
      removingDelta = -distances[originalRoute[positionPickup - 1]][originalRoute[positionPickup]] -
                      distances[originalRoute[positionPickup]][originalRoute[positionPickup + 1]] -
                      distances[originalRoute[positionDelivery - 1]][originalRoute[positionDelivery]] -
                      distances[originalRoute[positionDelivery]][originalRoute[positionDelivery + 1]] +
                      distances[originalRoute[positionPickup - 1]][originalRoute[positionPickup + 1]] +
                      distances[originalRoute[positionDelivery - 1]][originalRoute[positionDelivery + 1]];

    } else {
      int idx_first = std::min(positionPickup, positionDelivery);
      int idx_next = std::max(positionPickup, positionDelivery);

      removingDelta = -distances[originalRoute[idx_first - 1]][originalRoute[idx_first]] -
                      distances[originalRoute[idx_first]][originalRoute[idx_next]] -
                      distances[originalRoute[idx_next]][originalRoute[idx_next + 1]] +
                      distances[originalRoute[idx_first - 1]][originalRoute[idx_next + 1]];
    }
  }

  // Precompute accumulated
  int bestPickup = -1;
  int bestDelivery = -1;
  int selectedPickup = -1;
  int selectedDelivery = -1;
  double bestDeliveryCost = DBL_MAX;
  double bestMoveCost = DBL_MAX;
  double selectedMoveCost = DBL_MAX;

  for (int i = routeSZ - 2; i > 0; i--) {
    double pickupCost = -distances[routeToWork[i - 1]][routeToWork[i]] +
                        distances[routeToWork[i - 1]][pickupNode->idx] +
                        distances[pickupNode->idx][routeToWork[i]];

    double deliveryCost = -distances[routeToWork[i]][routeToWork[i + 1]] +
                          distances[routeToWork[i]][pickupNode->pair] +
                          distances[pickupNode->pair][routeToWork[i + 1]];

    if (deliveryCost < bestDeliveryCost) {
      bestDeliveryCost = deliveryCost;
      bestDelivery = i + 1;
    }

    double currentMoveCost = pickupCost + bestDeliveryCost;

    if (currentMoveCost < bestMoveCost) {
      bestMoveCost = currentMoveCost;
      bestPickup = i;

      if (bestMoveCost < selectedMoveCost) {
        selectedMoveCost = bestMoveCost;

        selectedPickup = bestPickup;
        selectedDelivery = bestDelivery;
      }
    }
  }

  for (int i = 1; i < routeSZ; i++) {
    double currentMoveCost =
        distances[routeToWork[i - 1]][pickupNode->idx] + distances[pickupNode->idx][pickupNode->pair] +
        distances[pickupNode->pair][routeToWork[i]] - distances[routeToWork[i - 1]][routeToWork[i]];

    if (currentMoveCost < selectedMoveCost) {
      selectedMoveCost = currentMoveCost;
      selectedPickup = i;
      selectedDelivery = i;
    }
  }

  if (selectedMoveCost < DBL_MAX) {
    state.originPickup = positionPickup;
    state.originDelivery = positionDelivery;
    state.destinyPickup = selectedPickup;
    state.destinyDelivery = selectedDelivery;
    state.pickupNode = pickupNode;
    eval.cost = solution->cost + selectedMoveCost + removingDelta;
  }

  return eval;
}

double PDPRelocateMove::move(PDPSolution* solution, const PDPMoveEvaluation& eval) {
  PDPRelocateMoveState* curState = (PDPRelocateMoveState*)eval.moveparam;

  PDPRoute* route_A = &solution->route;
  PDPNode* node = curState->pickupNode;

  int P, D;

  P = std::min(curState->originDelivery, curState->originPickup);
  D = std::max(curState->originDelivery, curState->originPickup);
  if (P != -1) {
    route_A->erase(route_A->begin() + D);
    route_A->erase(route_A->begin() + P);
  }

  P = std::min(curState->destinyDelivery, curState->destinyPickup);
  D = std::max(curState->destinyDelivery, curState->destinyPickup);
  route_A->insert(route_A->begin() + D, node->pair);
  route_A->insert(route_A->begin() + P, node->idx);

  solution->cost = route_A->PrecomputeRouteInformation();

  return PDPMove::move(solution, eval);
}

std::string PDPRelocateMove::name() const {
  return "pdp_relocate";
}

}  // namespace moves
}  // namespace pdp
