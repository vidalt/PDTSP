#include "pdp4optmove.h"

#include "pdp/pdproute.h"
#include "utils/application.h"

using namespace std;
namespace pdp {
namespace moves {

PDP4optMove::PDP4optMove(bool allowInfeasible) {
  this->allowInfeasible = allowInfeasible;
  int instanceSize = Application::instance->Size();

  customerPosition.resize(instanceSize);
  subrouteInfo.resize(instanceSize + 1);
  for (size_t i = 0; i < subrouteInfo.size(); i++) {
    subrouteInfo[i].resize(subrouteInfo.size());
  }

  oldSol = new int[instanceSize + 1];
  sol = new int[instanceSize + 1];

  cost = new double *[instanceSize];
  best_reach = new double_pair[instanceSize];
  best_cross = new double_pair[instanceSize];
  pred_reach = new int_pair[instanceSize];
  pred_cross = new ac_pair[instanceSize];

  for (int i = 0; i < instanceSize; i++) {
    cost[i] = new double[instanceSize];
  }

  countDD = 0;
  countDC = 0;
  countCD = 0;
  countTotalDD = 0;
  countTotalDC = 0;
  countTotalCD = 0;
}

PDP4optMove::~PDP4optMove() {
  int instanceSize = Application::instance->Size();
  for (int i = 0; i < instanceSize; i++) {
    delete[] cost[i];
  }
  delete[] pred_cross;
  delete[] pred_reach;
  delete[] best_reach;
  delete[] best_cross;
  delete[] cost;
  delete[] sol;
  delete[] oldSol;
}

size_t PDP4optMove::ResetCount() {
  size_t ret = PDPMove::ResetCount();
  countDD = 0;
  countDC = 0;
  countCD = 0;

  return ret;
}

const char *PDP4optMove::ExtraTotalInfo() const {
  if (countTotalDC + countTotalCD + countTotalDD) {
    sprintf((char *)extraInfoBuffer, "dc=%zu;cd=%zu;dd=%zu", countTotalDC, countTotalCD, countTotalDD);
    return extraInfoBuffer;
  } else {
    return 0;
  }
}

const char *PDP4optMove::ExtraInfo() const {
  if (countDD + countDC + countCD) {
    sprintf((char *)extraInfoBuffer, "dc=%zu,cd=%zu,dd=%zu", countDC, countCD, countDD);
    return extraInfoBuffer;
  } else {
    return 0;
  }
}

void PDP4optMove::Precompute(PDPRoute *r) {
  register PDPNode **nodes = static_cast<PDPNode **>(Application::instance->Data());

  PDPRoute &route = *r;
  int sz = route.size();

  for (int i = 0; i < sz; i++) {
    customerPosition[route[i]] = i;
  }

  // subroute infos
  for (int i = 0; i < sz; i++) {
    SubrouteInfo info;
    PickupDeliveryInfo *pRange;
    info.reversible = true;

    for (int j = i; j < sz; j++) {
      PDPNode &node = *nodes[route[j]];

      if (node.idx != 0) {
        int &pairPos = customerPosition[node.pair];

        if (pairPos >= i && pairPos <= j) {
          info.reversible = false;
        } else {
          if (pairPos < i) {
            pRange = &info.before;

            if (node.isDelivery) {
              if (pRange->early > pairPos || pRange->early == -1) pRange->early = pairPos;
              if (pRange->later < pairPos) pRange->later = pairPos;
            }
          }
        }
      }
      subrouteInfo[i][j] = info;
    }
  }
}

#define validateCC(i1, j1) (!allowInfeasible && subrouteInfo[i1 + 1][j1].reversible)

#define validateDD(i1, j1, i2, j2)                                                                \
  (allowInfeasible || (Application::ls_4opt_dd && (subrouteInfo[i2 + 1][j2].before.later <= i1 && \
                                                   subrouteInfo[j1 + 1][j2].before.later <= i1)))

#define validateCD(i1, j1, i2, j2)                                                                \
  (!allowInfeasible && (Application::ls_4opt_cd && subrouteInfo[i2 + 1][j2].before.later <= i1 && \
                        subrouteInfo[i2 + 1][j1].reversible && subrouteInfo[j1 + 1][j2].reversible))

#define validateDC(i1, j1, i2, j2)                                                                \
  (!allowInfeasible && (Application::ls_4opt_dc && subrouteInfo[j1 + 1][j2].before.later <= i1 && \
                        subrouteInfo[i1 + 1][i2].reversible && subrouteInfo[i2 + 1][j1].reversible))

#define SimpleTerminalNodeUpdate                   \
  {                                                \
    if (cost[i][j] < best_t && validateCC(i, j)) { \
      best_t = cost[i][j];                         \
      pred_t[1].i = i;                             \
      pred_t[1].j = j;                             \
      type[0] = 0;                                 \
      type[1] = 0;                                 \
      searchState.segments.n = 0;                  \
      searchState.segments.type = MOVE_CC;         \
    }                                              \
  }

#define BestTerminalNodeUpdate                                            \
  {                                                                       \
    /* y=d  and  x=d */                                                   \
    if ((cost[j][i] + best_cross[j - 1].d) < best_t &&                    \
        validateDD(pred_cross[j - 1].d.i, pred_cross[j - 1].d.j, i, j)) { \
      best_t = cost[j][i] + best_cross[j - 1].d;                          \
      pred_t[0] = pred_cross[j - 1].d;                                    \
      pred_t[1].i = i;                                                    \
      pred_t[1].j = j;                                                    \
      type[0] = 1;                                                        \
      type[1] = 1;                                                        \
      searchState.segments.n = 0;                                         \
      searchState.segments.type = MOVE_DD;                                \
    }                                                                     \
                                                                          \
    /* y=c and x=d  */                                                    \
    if ((cost[j][i] + best_cross[j - 1].c) < best_t &&                    \
        validateCD(pred_cross[j - 1].c.i, pred_cross[j - 1].c.j, i, j)) { \
      best_t = cost[j][i] + best_cross[j - 1].c;                          \
      pred_t[0] = pred_cross[j - 1].c;                                    \
      pred_t[1].i = i;                                                    \
      pred_t[1].j = j;                                                    \
      type[0] = 1;                                                        \
      type[1] = 0;                                                        \
      searchState.segments.n = 0;                                         \
      searchState.segments.type = MOVE_CD;                                \
    }                                                                     \
                                                                          \
    /* y=d and x=c */                                                     \
    if ((cost[i][j] + best_cross[j - 1].d) < best_t &&                    \
        validateDC(pred_cross[j - 1].d.i, pred_cross[j - 1].d.j, i, j)) { \
      best_t = cost[i][j] + best_cross[j - 1].d;                          \
      pred_t[0] = pred_cross[j - 1].d;                                    \
      pred_t[1].i = i;                                                    \
      pred_t[1].j = j;                                                    \
      type[0] = 0;                                                        \
      type[1] = 1;                                                        \
      searchState.segments.n = 0;                                         \
      searchState.segments.type = MOVE_DC;                                \
    }                                                                     \
  }

#define BestReachUpdate                 \
  {                                     \
    if (cost[i][j] < best_reach[j].c) { \
      best_reach[j].c = cost[i][j];     \
      pred_reach[j].c = i;              \
    }                                   \
                                        \
    if (cost[j][i] < best_reach[j].d) { \
      best_reach[j].d = cost[j][i];     \
      pred_reach[j].d = i;              \
    }                                   \
  }

#define BestCrossUpdate                          \
  {                                              \
    if (best_reach[j].c < best_cross[j - 1].c) { \
      best_cross[j].c = best_reach[j].c;         \
      pred_cross[j].c.i = pred_reach[j].c;       \
      pred_cross[j].c.j = j;                     \
    } else {                                     \
      best_cross[j].c = best_cross[j - 1].c;     \
      pred_cross[j].c = pred_cross[j - 1].c;     \
    }                                            \
                                                 \
    if (best_reach[j].d < best_cross[j - 1].d) { \
      best_cross[j].d = best_reach[j].d;         \
      pred_cross[j].d.i = pred_reach[j].d;       \
      pred_cross[j].d.j = j;                     \
    } else {                                     \
      best_cross[j].d = best_cross[j - 1].d;     \
      pred_cross[j].d = pred_cross[j - 1].d;     \
    }                                            \
  }

void PDP4optMove::doMove(int *sol, int n, Segments segments) {
  int c = 0;
  memcpy(oldSol, sol, sizeof(int) * (n + 1));

  int npts = segments.n * 2;
  for (int seg = 0; seg < npts; seg += 2) {
    int s = segments.points[seg];
    int e = segments.points[seg + 1];

    if (s <= e) {
      for (int i = s; i <= e; i++) {
        sol[c++] = oldSol[i];
      }
    } else {
      for (int i = s; i >= e; i--) {
        sol[c++] = oldSol[i];
      }
    }
  }
}

double PDP4optMove::connectSegmentsDelta(const int *sol, const int *a, const int *b, const int *c,
                                         const int *d, const int *e) {
  double **distances = Application::instance->Distances();
  return distances[sol[a[1]]][sol[b[0]]] + distances[sol[b[1]]][sol[c[0]]] + distances[sol[c[1]]][sol[d[0]]] +
         distances[sol[d[1]]][sol[e[0]]];
}

#define checkCombinations                                                                 \
                                                                                          \
  int bMax = subrouteInfo[blkB[0]][blkB[1]].reversible ? 2 : 1;                           \
  int cMax = subrouteInfo[blkC[0]][blkC[1]].reversible ? 2 : 1;                           \
  int dMax = subrouteInfo[blkD[0]][blkD[1]].reversible ? 2 : 1;                           \
                                                                                          \
  for (int b = 0; b < bMax; b++) {                                                        \
    const int *B = b ? blkb : blkB;                                                       \
    for (int c = 0; c < cMax; c++) {                                                      \
      const int *C = c ? blkc : blkC;                                                     \
      for (int d = 0; d < dMax; d++) {                                                    \
        const int *D = d ? blkd : blkD;                                                   \
                                                                                          \
        double cost = connectSegmentsDelta(sol, blkA, B, C, D, blkE) - removedEdgesDelta; \
        if (cost < bestKnown) {                                                           \
          bestKnown = cost;                                                               \
          searchState.segments.set(5, blkA, B, C, D, blkE, type);                         \
        }                                                                                 \
      }                                                                                   \
    }                                                                                     \
  }

double PDP4optMove::bestFromDD(const int *sol, int blks[][2], double removedEdgesDelta, double bestKnown) {
  // check this configuration violates the precedence rule?
  if (!(allowInfeasible ||
        (Application::ls_4opt_dd && (subrouteInfo[blks[2][0] + 1][blks[6][0]].before.later <= blks[0][1] &&
                                     subrouteInfo[blks[4][0] + 1][blks[6][0]].before.later <= blks[0][1]))))
    return bestKnown;

  int bMax = subrouteInfo[blks[5][0]][blks[5][1]].reversible ? 2 : 1;
  int cMax = subrouteInfo[blks[3][0]][blks[3][1]].reversible ? 2 : 1;
  int dMax = subrouteInfo[blks[1][0]][blks[1][1]].reversible ? 2 : 1;

  for (int b = 0; b < bMax; b++) {
    const int *B = blks[5 + b];
    for (int c = 0; c < cMax; c++) {
      const int *C = blks[3 + c];
      for (int d = 0; d < dMax; d++) {
        const int *D = blks[1 + d];

        double cost = connectSegmentsDelta(sol, blks[0], B, C, D, blks[7]) - removedEdgesDelta;
        if (cost < bestKnown) {
          bestKnown = cost;
          searchState.segments.set(5, blks[0], B, C, D, blks[7], MOVE_DD);
        }
      }
    }
  }

  return bestKnown;
}

double PDP4optMove::bestFromCD(const int *sol, int blks[][2], double removedEdgesDelta, double bestKnown) {
  // check this configuration violates the precedence rule?
  if (!(allowInfeasible ||
        (Application::ls_4opt_cd && subrouteInfo[blks[3][0]][blks[6][0]].before.later <= blks[0][1])))
    return bestKnown;

  int bMax = subrouteInfo[blks[3][0]][blks[3][1]].reversible ? 2 : 1;
  int cMax = subrouteInfo[blks[5][0]][blks[5][1]].reversible ? 2 : 1;
  int dMax = subrouteInfo[blks[1][0]][blks[1][1]].reversible ? 2 : 1;

  for (int b = 0; b < bMax; b++) {
    const int *B = blks[3 + b];
    for (int c = 0; c < cMax; c++) {
      const int *C = blks[5 + c];
      for (int d = 0; d < dMax; d++) {
        const int *D = blks[1 + d];

        double cost = connectSegmentsDelta(sol, blks[0], B, C, D, blks[7]) - removedEdgesDelta;
        if (cost < bestKnown) {
          bestKnown = cost;
          searchState.segments.set(5, blks[0], B, C, D, blks[7], MOVE_CD);
        }
      }
    }
  }

  return bestKnown;
}

double PDP4optMove::bestFromDC(const int *sol, int blks[][2], double removedEdgesDelta, double bestKnown) {
  // check this configuration violates the precedence rule?
  if (!(allowInfeasible ||
        (Application::ls_4opt_dc && subrouteInfo[blks[5][0]][blks[5][1]].before.later <= blks[0][1])))
    return bestKnown;

  int bMax = subrouteInfo[blks[5][0]][blks[5][1]].reversible ? 2 : 1;
  int cMax = subrouteInfo[blks[1][0]][blks[1][1]].reversible ? 2 : 1;
  int dMax = subrouteInfo[blks[3][0]][blks[3][1]].reversible ? 2 : 1;

  for (int b = 0; b < bMax; b++) {
    const int *B = blks[5 + b];
    for (int c = 0; c < cMax; c++) {
      const int *C = blks[1 + c];
      for (int d = 0; d < dMax; d++) {
        const int *D = blks[3 + d];

        double cost = connectSegmentsDelta(sol, blks[0], B, C, D, blks[7]) - removedEdgesDelta;
        if (cost < bestKnown) {
          bestKnown = cost;
          searchState.segments.set(5, blks[0], B, C, D, blks[7], MOVE_DC);
        }
      }
    }
  }

  return bestKnown;
}

double PDP4optMove::best4opt(int *sol, int n, int i1, int j1, int i2, int j2, double bestKnown) {
  int blks[][2] = {{0, i1},      {i1 + 1, i2}, {i2, i1 + 1}, {i2 + 1, j1},
                   {j1, i2 + 1}, {j1 + 1, j2}, {j2, j1 + 1}, {j2 + 1, n}};

  double **distances = Application::instance->Distances();
  double removedEdgesDelta = distances[sol[i1]][sol[i1 + 1]] + distances[sol[j1]][sol[j1 + 1]] +
                             distances[sol[i2]][sol[i2 + 1]] + distances[sol[j2]][sol[j2 + 1]];

  bestKnown = bestFromDD(sol, blks, removedEdgesDelta, bestKnown);
  bestKnown = bestFromCD(sol, blks, removedEdgesDelta, bestKnown);
  bestKnown = bestFromDC(sol, blks, removedEdgesDelta, bestKnown);

  return bestKnown;
}

PDPMoveEvaluation PDP4optMove::Evaluate(PDPSolution * /*solution*/, PDPNode * /*pickupNode*/) {
  PDPMoveEvaluation eval;
  eval.neighborhood = this;
  eval.moveparam = 0;
  eval.cost = DBL_MAX;

  return eval;
}

PDPMoveEvaluation PDP4optMove::Evaluate(PDPSolution *solution) {
  return Evaluate(solution, false);
}

PDPMoveEvaluation PDP4optMove::Evaluate(PDPSolution *solution, bool unfeasible) {
  size_t i, j;
  allowInfeasible = unfeasible;

  PDPMoveEvaluation eval;
  eval.neighborhood = this;
  eval.moveparam = &searchState;
  searchState.segments.n = 0;

  PDPRoute *r = &solution->route;
  Precompute(r);

  int *route = sol;
  for (size_t i = 0; i < r->size(); i++) {
    route[i] = (*r)[i];
  }

  int n = r->size() - 1;

  /// Compute 2AC costs
  int prev_i, next_i, prev_j, next_j;
  double **distance = Application::instance->Distances();
  register double *costi = nullptr;

  // Number of nodes of hamiltonian cycle=n+1; Number of edges will
  // be equal to nodes of hamiltonian cycle -1 (n)
  for (int i = 0; i < n - 2; i++) {
    prev_i = route[i];
    next_i = route[i + 1];
    costi = cost[i];
    register double *disPrevi = distance[prev_i];
    register double *disNexti = distance[next_i];

    for (int j = i + 2; n - j; ++j) {
      prev_j = route[j];
      next_j = route[j + 1];
      double deltaR = (disPrevi[next_i] + distance[prev_j][next_j]);
      costi[j] = disPrevi[prev_j] + disNexti[next_j] - deltaR;
      cost[j][i] = disPrevi[next_j] + disNexti[prev_j] - deltaR;
    }
  }

  double &best_t = searchState.best_t;
  AlternatingCycle *pred_t = searchState.pred_t;
  bool *type = searchState.type;

  if (allowInfeasible) {
    best_t = 0;
  }

  // Step 1: Initialization.
  i = 0;
  best_t = DBL_MAX;
  for (j = 2; j < n - 1; j++) {
    best_reach[j].c = cost[i][j];  // best_reach_c(0,j) = cost_c(0,j)
    best_reach[j].d = cost[j][i];  // best_reach_d(0,j) = cost_d(0,j)
    pred_reach[j].c = 0;           // pred_reach_c = 0
    pred_reach[j].d = 0;           // pred_reach_d = 0
    SimpleTerminalNodeUpdate;
  }

  // Step 2: Main.
  for (i = 1; i < n - 2; i++) {
    best_cross[i + 1].c = best_reach[i + 1].c;    // c
    pred_cross[i + 1].c.i = pred_reach[i + 1].c;  // c
    pred_cross[i + 1].c.j = i + 1;                // c

    best_cross[i + 1].d = best_reach[i + 1].d;    // d
    pred_cross[i + 1].d.i = pred_reach[i + 1].d;  // d
    pred_cross[i + 1].d.j = i + 1;                // d

    for (j = i + 2; j < n; j++) {
      BestTerminalNodeUpdate;
      SimpleTerminalNodeUpdate;

      if (j < n - 1) {
        BestCrossUpdate;
        BestReachUpdate;
      }
    }
  }

  eval.cost = solution->cost + best_t;
  return eval;
}

#define checkStar(i1, i2, j1, j2, firstImprovement)             \
  {                                                             \
    ac1d = cost[j1][i1];                                        \
    ac1c = cost[i1][j1];                                        \
    ac2d = cost[j2][i2];                                        \
    ac2c = cost[i2][j2];                                        \
                                                                \
    /* x=d  and  y=d */                                         \
    if ((ac1d + ac2d) < best_t && validateDD(i1, j1, i2, j2)) { \
      best_t = ac1d + ac2d;                                     \
      pred_t[0].i = i1;                                         \
      pred_t[0].j = j1;                                         \
      pred_t[1].i = i2;                                         \
      pred_t[1].j = j2;                                         \
      type[0] = 1;                                              \
      type[1] = 1;                                              \
      if (firstImprovement) return best_t;                      \
    }                                                           \
                                                                \
    /* x=d  and  y=c */                                         \
    if ((ac1c + ac2d) < best_t && validateCD(i1, j1, i2, j2)) { \
      best_t = ac1c + ac2d;                                     \
      pred_t[0].i = i1;                                         \
      pred_t[0].j = j1;                                         \
      pred_t[1].i = i2;                                         \
      pred_t[1].j = j2;                                         \
      type[0] = 1;                                              \
      type[1] = 0;                                              \
      if (firstImprovement) return best_t;                      \
    }                                                           \
                                                                \
    /* x=c  and  y=d */                                         \
    if ((ac1d + ac2c) < best_t && validateDC(i1, j1, i2, j2)) { \
      best_t = ac1d + ac2c;                                     \
      pred_t[0].i = i1;                                         \
      pred_t[0].j = j1;                                         \
      pred_t[1].i = i2;                                         \
      pred_t[1].j = j2;                                         \
      type[0] = 0;                                              \
      type[1] = 1;                                              \
      if (firstImprovement) return best_t;                      \
    }                                                           \
  }

double PDP4optMove::move(PDPSolution *solution, const PDPMoveEvaluation &eval) {
  PDPRoute *r = &solution->route;
  PDP4optMoveState &state = *(PDP4optMoveState *)eval.moveparam;
  int n = r->size() - 1;

  if (state.segments.n == 0) {
    int &i1 = state.pred_t[0].i;
    int &j1 = state.pred_t[0].j;
    int &i2 = state.pred_t[1].i;
    int &j2 = state.pred_t[1].j;
    const int blkA[] = {0, i1};
    const int blkB[] = {i1 + 1, i2};
    const int blkC[] = {i2 + 1, j1};
    const int blkD[] = {j1 + 1, j2};
    const int blkE[] = {j2 + 1, n};
    const int blkb[] = {i2, i1 + 1};
    const int blkc[] = {j1, i2 + 1};
    const int blkd[] = {j2, j1 + 1};
    const int blkAcc[] = {0, i2};
    const int blkcc[] = {j2, i2 + 1};

    switch (state.segments.type) {
      case MOVE_CC:
        state.segments.set(3, blkAcc, blkcc, blkE, 0, 0, MOVE_CC);
        break;

      case MOVE_DD:
        state.segments.set(5, blkA, blkD, blkC, blkB, blkE, MOVE_DD);
        break;

      case MOVE_DC:
        state.segments.set(5, blkA, blkD, blkb, blkc, blkE, MOVE_DC);
        break;

      case MOVE_CD:
        state.segments.set(5, blkA, blkc, blkd, blkB, blkE, MOVE_CD);
        break;

      default:
        break;
    }
  }

  switch (state.segments.type) {
    case MOVE_CC:
      break;

    case MOVE_DD:
      countDD++;
      countTotalDD++;
      break;

    case MOVE_DC:
      countDC++;
      countTotalDC++;
      break;

    case MOVE_CD:
      countCD++;
      countTotalCD++;
      break;

    default:
      break;
  }

  doMove(r->data(), n, state.segments);

  solution->cost -= r->Cost();
  solution->cost += r->PrecomputeRouteInformation();

  return PDPMove::move(solution, eval);
}

std::string PDP4optMove::name() const {
  return "pdp_4-opt";
}

}  // namespace moves
}  // namespace pdp
