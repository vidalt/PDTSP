#include "pdporoptmove.h"

#include "pdp/pdproute.h"
#include "utils/application.h"

namespace pdp {
namespace moves {

PDPOroptMove::PDPOroptMove() : countFast(0), countSlow(0), countTotalFast(0), countTotalSlow(0) {
  numberOfNodes = Application::instance->Size();
  visited = new bool[numberOfNodes];
  positions = new int[numberOfNodes + 2];

  countFast = 0;
  countSlow = 0;
  countTotalFast = 0;
  countTotalSlow = 0;
}

PDPOroptMove::~PDPOroptMove() {
  delete[] visited;
  delete[] positions;
}

#define updateSelectedMove(fst)                                                                        \
  double insertionCost = removingDelta + -distances[(*r)[pos - 1]][(*r)[pos]] +                        \
                         distances[(*r)[pos - 1]][(*r)[s]] + distances[(*r)[e]][(*r)[pos]];            \
                                                                                                       \
  double revInsertionCost = !reversible                                                                \
                                ? DBL_MAX                                                              \
                                : (removingDelta + -distances[(*r)[pos - 1]][(*r)[pos]] +              \
                                   distances[(*r)[s]][(*r)[pos]] + distances[(*r)[pos - 1]][(*r)[e]]); \
                                                                                                       \
  if (insertionCost < eval.cost && insertionCost <= revInsertionCost) {                                \
    eval.cost = insertionCost;                                                                         \
    state.reversal = false;                                                                            \
    state.blockS = s;                                                                                  \
    state.blockE = e;                                                                                  \
    state.insertBefore = pos;                                                                          \
  } else {                                                                                             \
    if (revInsertionCost < eval.cost && revInsertionCost < insertionCost) {                            \
      eval.cost = revInsertionCost;                                                                    \
      state.reversal = true;                                                                           \
      state.blockS = s;                                                                                \
      state.blockE = e;                                                                                \
      state.insertBefore = pos;                                                                        \
    }                                                                                                  \
  }

PDPMoveEvaluation PDPOroptMove::Evaluate(PDPSolution* solution, PDPNode* pickupNode) {
  PDPNode** nodes = static_cast<PDPNode**>(Application::instance->Data());

  PDPMoveEvaluation eval;
  eval.cost = DBL_MAX;
  eval.neighborhood = this;
  eval.moveparam = &state;
  state.fastMove = true;

  double** distances = Application::instance->Distances();
  PDPRoute* r = &solution->route;
  int n = static_cast<int>(r->size());

  for (size_t i = 0; i < r->size() - 1; i++) {
    positions[(*r)[i]] = i;
  }

  for (int i = 0; i <= 1; i++) {
    size_t s = i ? positions[pickupNode->idx] : positions[pickupNode->pair];
    bool reversible = true;

    PDPNode* node = nodes[(*r)[s]];
    size_t posPair;
    for (size_t e = (Application::ls_relocate ? s + 1 : s); (e < n - 1) && (e - s <= Application::or_k);
         e++) {
      node = nodes[(*r)[e]];
      reversible = reversible && (node->isPickup || positions[node->pair] < s);

      double removingDelta = distances[(*r)[s - 1]][(*r)[e + 1]] - distances[(*r)[s - 1]][(*r)[s]] -
                             distances[(*r)[e]][(*r)[e + 1]];

      for (size_t pos = s - 1; pos > 0; pos--) {
        node = nodes[(*r)[pos]];
        posPair = positions[node->pair];
        if (node->isPickup && posPair >= s && posPair <= e)  // violating precedence
          break;

        updateSelectedMove(false);
      }

      for (int pos = e + 2; pos < n; pos++) {
        node = nodes[(*r)[pos - 1]];
        posPair = positions[node->pair];
        if (node->isDelivery && posPair >= s && posPair <= e)  // violating precedence
          break;

        updateSelectedMove(false);
      }

      if (eval.cost < -0.01) break;
    }
  }

  if (eval.cost < -0.01) {
    eval.cost += solution->cost;
  } else {
    eval.cost = DBL_MAX;
  }

  return eval;
}

PDPMoveEvaluation PDPOroptMove::Evaluate(PDPSolution* /*solution*/) {
  PDPMoveEvaluation eval;
  eval.cost = DBL_MAX;
  eval.neighborhood = this;
  eval.moveparam = &state;
  state.fastMove = false;

  return eval;
}

double PDPOroptMove::move(PDPSolution* solution, const PDPMoveEvaluation& eval) {
  PDPBlockRelocateMoveState* curState = (PDPBlockRelocateMoveState*)eval.moveparam;
  PDPRoute* route = &solution->route;

  int n = (int)route->size();
  memcpy(positions, route->data(), sizeof(int) * n);

  if (curState->fastMove) {
    countFast++;
    countTotalFast++;
  } else {
    countSlow++;
    countTotalSlow++;
  }

  size_t c = 0;
  if (curState->insertBefore < curState->blockS) {
    for (int i = 0; i < curState->insertBefore; i++)
      (*route)[c++] = positions[i];

    if (curState->reversal)
      for (int i = curState->blockE; i >= curState->blockS; i--)
        (*route)[c++] = positions[i];
    else
      for (int i = curState->blockS; i <= curState->blockE; i++)
        (*route)[c++] = positions[i];

    for (int i = curState->insertBefore; i < curState->blockS; i++)
      (*route)[c++] = positions[i];

    for (int i = curState->blockE + 1; i < n; i++)
      (*route)[c++] = positions[i];
  } else {
    for (int i = 0; i < curState->blockS; i++)
      (*route)[c++] = positions[i];

    for (int i = curState->blockE + 1; i < curState->insertBefore; i++)
      (*route)[c++] = positions[i];

    if (curState->reversal)
      for (int i = curState->blockE; i >= curState->blockS; i--)
        (*route)[c++] = positions[i];
    else
      for (int i = curState->blockS; i <= curState->blockE; i++)
        (*route)[c++] = positions[i];

    for (int i = curState->insertBefore; i < n; i++)
      (*route)[c++] = positions[i];
  }

  solution->cost -= route->Cost();
  solution->cost += route->PrecomputeRouteInformation();

  return PDPMove::move(solution, eval);
}

std::string PDPOroptMove::name() const {
  return "pdp_or-opt";
}

size_t PDPOroptMove::ResetCount() {
  size_t ret = PDPMove::ResetCount();
  countFast = 0;
  countSlow = 0;

  return ret;
}

const char* PDPOroptMove::ExtraTotalInfo() const {
  if (countTotalFast + countTotalSlow) {
    sprintf((char*)extraInfoBuffer, "fst=%lu;slw=%lu", countTotalFast, countTotalSlow);
    return extraInfoBuffer;
  } else {
    return nullptr;
  }
}

const char* PDPOroptMove::ExtraInfo() const {
  if (countFast + countSlow) {
    sprintf((char*)extraInfoBuffer, "fst=%lu,slw=%lu", countFast, countSlow);
    return extraInfoBuffer;
  } else {
    return nullptr;
  }
}

}  // namespace moves
}  // namespace pdp
