#include "pdpmoveevaluation.h"

#include "pdpmove.h"

namespace pdp {
namespace moves {

bool PDPMoveEvaluation::Apply(PDPSolution* solution, bool force) const {
  bool isBestMove = (solution->cost - cost) > 0.01;

  if (neighborhood != nullptr && (force || isBestMove)) {
    neighborhood->move(solution, *this);
    return true;
  }

  return false;
}

}  // namespace moves
}  // namespace pdp
