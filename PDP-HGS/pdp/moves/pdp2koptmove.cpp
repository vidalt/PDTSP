#include "pdp2koptmove.h"

#include "pdp/pdproute.h"
#include "utils/application.h"

using namespace std;

namespace pdp {
namespace moves {

k2opmem infk2 = k2opmem(0, 0, 0, DBL_MAX);
inline k2opmem& Best(k2opmem& a, k2opmem& b) {
  return (a.cost < b.cost) ? a : b;
}

k2opmem& Best(k2opmem* m1, k2opmem* m2) {
  return Best(*m1, *m2);
}

void ApplyK2opt(int* s, k2opmem* bestMove, int& count) {
  if (bestMove->next) {
    ApplyK2opt(s, bestMove->next, count);
  }

  if (bestMove->I != bestMove->J) {
    std::reverse(&s[bestMove->I + 1], &s[bestMove->J + 1]);
    count++;
  }
}

PDP2koptMove::PDP2koptMove(bool allowInfeasible) {
  this->allowInfeasible = allowInfeasible;
  visited = new bool[Application::instance->Size()];
  positions = new int[Application::instance->Size()];
  count2opt = 0;
  totalCount2opt = 0;

  int n = Application::instance->Size();

  mem = new k2opmem**[n];
  for (int i = 0; i < n; i++) {
    mem[i] = new k2opmem*[n];
    for (int j = 0; j < n; j++) {
      mem[i][j] = new k2opmem[2];
    }
  }
}

PDP2koptMove::~PDP2koptMove() {
  delete[] visited;
  delete[] positions;

  int n = Application::instance->Size();
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++)
      delete[] mem[i][j];
    delete[] mem[i];
  }
  delete[] mem;
}

PDPMoveEvaluation PDP2koptMove::Evaluate(PDPSolution*, PDPNode*) {
  PDPMoveEvaluation eval;
  eval.neighborhood = this;
  eval.moveparam = 0;
  eval.cost = DBL_MAX;

  return eval;
}

k2opmem k2inf(0, 0, 0, DBL_MAX);

PDPMoveEvaluation PDP2koptMove::Evaluate(PDPSolution* solution) {
  PDPMoveEvaluation eval;
  eval.neighborhood = this;

  PDPRoute* r = &solution->route;
  int n = r->size() - 1;
  int* s = r->data();
  for (int i = 0; i < n; i++) {
    positions[s[i]] = i;
  }

  PDPNode** instance = (PDPNode**)(Application::instance->Data());
  double** distance = Application::instance->Distances();

  // Base case to i == j
  for (int i = 0; i < n; i++) {
    bool ispair = instance[s[i]]->pair == s[i + 1];
    bool ispairRev = ispair && instance[s[i]]->isDelivery;

    const int j = i;
    mem[i][j][1] = k2opmem(0, 0, 0, 0);
    mem[i][j][0] = (!allowInfeasible && ispair) ? k2opmem(0, 0, 0, DBL_MAX) : k2opmem(0, 0, 0, 0);

    if (ispairRev) {
      std::swap(mem[i][j][1], mem[i][j][0]);
    }
  }

  // Base case to j - i == 1;
  for (int i = 0; i < n - 1; i++) {
    const int j = i + 1;
    bool ispair = instance[s[i]]->pair == s[i + 1] || instance[s[j]]->pair == s[j + 1] ||
                  instance[s[i]]->pair == s[j + 1];
    bool ispairRev = (instance[s[i]]->pair == s[i + 1] && instance[s[i]]->isDelivery) ||
                     (instance[s[j]]->pair == s[j + 1] && instance[s[j]]->isDelivery) ||
                     (instance[s[i]]->pair == s[j + 1] && instance[s[i]]->isDelivery);

    mem[i][j][1] = k2opmem(0, 0, 0, 0);
    mem[i][j][0] = (!allowInfeasible && ispair) ? k2opmem(0, 0, 0, DBL_MAX) : k2opmem(0, 0, 0, 0);

    if (ispairRev) {
      std::swap(mem[i][j][1], mem[i][j][0]);
    }
  }

  for (int k = 2; k < n; k++) {
    for (int i = 0; i < n - k; i++) {
      int j = i + k;

      double crossDelta = distance[s[i]][s[j]] + distance[s[i + 1]][s[j + 1]] -
                          (distance[s[i]][s[i + 1]] + distance[s[j]][s[j + 1]]);
      int pairPosI = positions[instance[s[i + 1]]->pair];
      int pairPosJ = positions[instance[s[j]]->pair];
      bool pairIinside = (!allowInfeasible && (pairPosI > i && pairPosI <= j));
      bool pairJinside = (!allowInfeasible && (pairPosJ > i && pairPosJ <= j));

      if (pairIinside || pairJinside) {
        mem[i][j][1] = Best(mem[i + 1][j][1], mem[i][j - 1][1]);
        mem[i][j][0] = k2opmem(i, j, &mem[i + 1][j - 1][1], mem[i + 1][j - 1][1].cost + crossDelta);
        k2opmem& a = (pairIinside) ? k2inf : mem[i + 1][j][0];
        k2opmem& b = (pairJinside) ? k2inf : mem[i][j - 1][0];
        mem[i][j][0] = Best(mem[i][j][0], Best(a, b));
      } else {
        k2opmem Tc_fw = k2opmem(i, j, &mem[i + 1][j - 1][0], mem[i + 1][j - 1][0].cost + crossDelta);
        k2opmem Tc_bw = k2opmem(i, j, &mem[i + 1][j - 1][1], mem[i + 1][j - 1][1].cost + crossDelta);

        mem[i][j][1] = Best(Tc_fw, Best(mem[i + 1][j][1], mem[i][j - 1][1]));
        mem[i][j][0] = Best(Tc_bw, Best(mem[i + 1][j][0], mem[i][j - 1][0]));
      }
    }
  }

  k2opmem& Tbest = mem[0][n - 1][1];

  if (Tbest.I == Tbest.J) {
    Tbest.cost = DBL_MAX;
  }

  eval.cost = solution->cost + Tbest.cost;
  eval.moveparam = &Tbest;

  return eval;
}

size_t PDP2koptMove::ResetCount() {
  size_t ret = PDPMove::ResetCount();
  count2opt = 0;
  return ret;
}

const char* PDP2koptMove::ExtraTotalInfo() const {
  sprintf((char*)extraInfoBuffer, "2-opt=%zu", totalCount2opt);
  return extraInfoBuffer;
}

const char* PDP2koptMove::ExtraInfo() const {
  sprintf((char*)extraInfoBuffer, "%zu", count2opt);
  return extraInfoBuffer;
}

double PDP2koptMove::move(PDPSolution* solution, const PDPMoveEvaluation& eval) {
  k2opmem* Tbest = (k2opmem*)eval.moveparam;
  PDPRoute* r = &solution->route;

  int count = 0;
  ApplyK2opt(r->data(), Tbest, count);
  count2opt += count;
  totalCount2opt += count;

  solution->cost -= r->Cost();
  solution->cost += r->PrecomputeRouteInformation();

  return PDPMove::move(solution, eval);
}

std::string PDP2koptMove::name() const {
  return "pdp_2k-opt";
}

}  // namespace moves
}  // namespace pdp
