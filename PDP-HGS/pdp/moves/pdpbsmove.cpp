#include "pdpbsmove.h"

#include "pdp/pdproute.h"
#include "utils/application.h"

using namespace std;
namespace pdp {
namespace moves {

PDPBsMove::PDPBsMove() {
  routeToWork = new int[Application::instance->Size() + 2];
  bs = new BSGraph(Application::bs_k, Application::instance->Size(), Application::instance->Distances(),
                   (clock_t)-1, (size_t)-1);

  state.changedroute.reserve(Application::instance->Size() + 2);
}

PDPBsMove::~PDPBsMove() {
  delete[] routeToWork;
  delete bs;
}

PDPMoveEvaluation PDPBsMove::Evaluate(PDPSolution *solution) {
  PDPMoveEvaluation eval;
  eval.cost = DBL_MAX;
  eval.neighborhood = this;
  eval.moveparam = &state;
  state.updatedcost = DBL_MAX;
  state.changedroute = solution->route;
  bs->updateOrder(state.changedroute, state.updatedcost);
  eval.cost = state.updatedcost;

  return eval;
}

PDPMoveEvaluation PDPBsMove::Evaluate(PDPSolution *, PDPNode *) {
  PDPMoveEvaluation eval;
  eval.cost = DBL_MAX;
  eval.neighborhood = this;
  eval.moveparam = &state;

  return eval;
}

double PDPBsMove::move(PDPSolution *solution, const PDPMoveEvaluation &eval) {
  PDPBsMoveState *curState = (PDPBsMoveState *)eval.moveparam;

  PDPRoute *route_A = &solution->route;

  *((vector<int> *)route_A) = curState->changedroute;

  solution->cost -= route_A->Cost();
  solution->cost += route_A->PrecomputeRouteInformation();

  return PDPMove::move(solution, eval);
}

std::string PDPBsMove::name() const {
  return "pdp-b&s";
}

}  // namespace moves
}  // namespace pdp
