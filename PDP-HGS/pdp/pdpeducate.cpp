#include "pdpeducate.h"

#include <stdio.h>

#include <algorithm>
#include <iostream>

#include "pdp/moves/pdpmove.h"
#include "pdp/pdproute.h"
#include "utils/application.h"
#include "utils/random.h"

namespace pdp {

Educate::Educate() {
  size_t sz = Application::instance->Size();
  PDPNode** nodes = (PDPNode**)Application::instance->Data();

  for (size_t i = 0; i < sz; i++) {
    if (nodes[i]->isPickup) {
      pickupNodes.push_back(nodes[i]);
    }
  }

  educateTotalCount = educateCount = 0;
}

Educate::~Educate() {
  for (unsigned int i = 0; i < size(); i++) {
    delete at(i);
  }
}

bool Educate::Run(PDPSolution* solution) {
  bool improved;
  bool useSlowNeighborhoods = Random::RandomReal() < Application::slow_nb_percentage;

  do {
    improved = FastNeighborhoods(solution);

    if (Application::Timeout()) break;

    if (useSlowNeighborhoods) {
      improved = improved | SlowNeighborhoods(solution);
    }
  } while (improved);
  solution->Recompute();

  educateCount++;
  educateTotalCount++;
  return true;
}

//! Perform Fast Neighborhood Local Search
bool Educate::FastNeighborhoods(PDPSolution* solution) {
  bool improved = false;
  random_shuffle(pickupNodes.begin(), pickupNodes.end());

  for (PDPNode* pickupNode : pickupNodes) {
    // Best route insert for this P-D pair.
    pdp::moves::PDPMoveEvaluation bestMove = EvaluateBestNeighborhood(solution, pickupNode);

    // Apply move
    improved |= bestMove.Apply(solution, false);

    if (Application::Timeout()) break;
  }
  return improved;
}

bool Educate::SlowNeighborhoods(PDPSolution* solution) {
  pdp::moves::PDPMoveEvaluation current;
  current = EvaluateBestNeighborhood(solution);
  return current.Apply(solution, false);
}

pdp::moves::PDPMoveEvaluation Educate::EvaluateBestNeighborhood(PDPSolution* solution) {
  pdp::moves::PDPMoveEvaluation best;
  best.cost = solution->cost;
  best.neighborhood = nullptr;

  Random::shuffle(this->begin(), this->end());
  for (Educate::iterator it = begin(); it != end(); it++) {
    clock_t startTime = clock();
    pdp::moves::PDPMoveEvaluation current = (*it)->Evaluate(solution);
    (*it)->AddCpuTime(1000 * static_cast<double>(clock() - startTime) / CLOCKS_PER_SEC);

    if (current.cost < best.cost) {
      best = current;
      if (Application::firstimprovement) break;
    }
  }

  return best;
}

pdp::moves::PDPMoveEvaluation Educate::EvaluateBestNeighborhood(PDPSolution* solution, PDPNode* pickupNode) {
  pdp::moves::PDPMoveEvaluation best;
  best.cost = solution->cost;
  best.neighborhood = nullptr;

  Random::shuffle(this->begin(), this->end());
  for (Educate::iterator it = begin(); it != end(); it++) {
    clock_t startTime = clock();
    pdp::moves::PDPMoveEvaluation current = (*it)->Evaluate(solution, pickupNode);
    (*it)->AddCpuTime(1000 * static_cast<double>(clock() - startTime) / CLOCKS_PER_SEC);
    if (current.cost < best.cost) {
      best = current;
      if (Application::firstimprovement) break;
    }
  }

  return best;
}

size_t Educate::ResetCounts() {
  size_t ret = educateCount;
  educateCount = 0;

  for (size_t i = 0; i < size(); i++) {
    at(i)->ResetCount();
  }

  return ret;
}

struct less_than_key_move {
    inline bool operator()(const pdp::moves::PDPMove* m1, const pdp::moves::PDPMove* m2) {
      return m1->name() < m2->name();
    }
};

std::string Educate::TotalMovesLog(bool percent) const {
  std::vector<pdp::moves::PDPMove*> tmpmoves = *this;
  std::sort(tmpmoves.begin(), tmpmoves.end(), less_than_key_move());

  const char* extra = nullptr;
  std::string s;
  char buff[100];

  for (size_t i = 0; i < size(); i++) {
    pdp::moves::PDPMove* move = tmpmoves.at(i);

    s += (s.size() ? ",\t  \"" : "  \"") + move->name() + "\": ";

    extra = move->ExtraTotalInfo();

    if (percent)
      sprintf(buff, "\"%.2lf%%\"", move->TotalCount() / (double)educateTotalCount);
    else {
      if (extra == nullptr) {
        sprintf(buff, "\"%zu - %.2lfs\"", move->TotalCount(), move->CpuTime() / 1000);
      } else {
        sprintf(buff, "\"%zu - (%s) - %.2lfs\"", move->TotalCount(), extra, move->CpuTime() / 1000);
      }
    }

    s += buff;
  }

  if (!percent) {
    sprintf(buff, ",\t  \"educate\": %zu", educateTotalCount);
    s += buff;
  }

  return s;
}

std::string Educate::MovesLog(bool percent) const {
  const char* extra = 0;
  std::string s;
  char buff[100];

  for (size_t i = 0; i < size(); i++) {
    pdp::moves::PDPMove* move = at(i);

    s += move->name() + ": ";

    if (percent)
      sprintf(buff, "%.2lf%%  ", move->Count() / (double)educateCount);
    else {
      extra = move->ExtraInfo();
      if (extra == 0) {
        sprintf(buff, "%lu  ", move->Count());
      } else {
        sprintf(buff, "%lu(%s) ", move->Count(), extra);
      }
    }

    s += buff;
  }

  if (!percent) {
    sprintf(buff, "(EDUCATE: %lu)", educateCount);
    s += buff;
  }

  return s;
}
}  // namespace pdp
