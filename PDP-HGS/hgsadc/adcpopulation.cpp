#include "adcpopulation.h"

#include <iostream>

#include "utils/application.h"
#include "utils/random.h"

namespace ga {

ADCPopulation::ADCPopulation() {
  this->populationSize = Application::hgsadc_populationSize;
  this->nclosest = Application::hgsadc_cl;

  nsz = std::max(populationSize * 5, populationSize + Application::hgsadc_offspringInGeneration);
  distanceMatrix = new double*[nsz];

  for (int i = 0; i < nsz; i++) {
    freeSolutions.push(nsz - 1 - i);
    distanceMatrix[i] = new double[nsz];

    for (int j = 0; j < nsz; j++)
      distanceMatrix[i][j] = DBL_MAX;
  }

  solutions = new Solution*[nsz];
  maxDistance = new double[nsz];
  rankingByCost = new int[nsz];
  rankingByDiversity = new int[nsz];
  rankingByDiversityAux = new int[nsz];
  distanceAux = new double[nsz];
  memset(solutions, 0, nsz * sizeof(Solution*));
  memset(rankingByCost, -1, nsz * sizeof(int));
  memset(maxDistance, (int)DBL_MAX, nsz * sizeof(double));

  for (int i = 0; i < nsz; i++) {
    rankingByDiversityAux[i] = rankingByDiversity[i] = i;
  }
}

Solution* ADCPopulation::at(size_t pos) const {
  return solutions[rankingByCost[pos]];
}

const Solution* ADCPopulation::operator[](size_t p) const {
  return at(p);
}

size_t ADCPopulation::size() const {
  size_t sz = nsz - freeSolutions.size();
  return sz;
}

ADCPopulation::~ADCPopulation() {
  delete[] distanceAux;
  delete[] maxDistance;
  delete[] rankingByDiversity;
  delete[] rankingByDiversityAux;
  delete[] rankingByCost;

  for (int i = 0; i < nsz; i++) {
    delete[] distanceMatrix[i];
    if (solutions[i] != 0) delete solutions[i];
  }

  delete[] solutions;
  delete[] distanceMatrix;
}

double ADCPopulation::DiversityContribution(int idx) const {
  Solution* s = solutions[idx];
  return (s == nullptr) ? -1.0 : s->dc;
}

void ADCPopulation::RecomputeFitness() {
  size_t nbIndiv = size();
  if (nbIndiv == 0) return;

  ///// UPDATE DIVERSITY RANKING ///////////
  std::sort(rankingByDiversityAux, rankingByDiversityAux + nsz,
            [this](int a, int b) { return DiversityContribution(a) > DiversityContribution(b); });

  for (int i = 0; i < nsz; i++)
    rankingByDiversity[rankingByDiversityAux[i]] = i;

  ///// UPDATE BIASED FITNESS /////////
  double divContribution = 1.0 - (double)Application::hgsadc_el / (double)nbIndiv;

  for (size_t i = 0; i < nbIndiv; i++) {
    double fitRank = (double)i / (nbIndiv - 1);
    double divRank = (double)rankingByDiversity[rankingByCost[i]] / (double)(nbIndiv - 1);

    solutions[rankingByCost[i]]->fitness = fitRank + divContribution * divRank;
  }
}

void ADCPopulation::RemoveClones() {
  if (size() < 2) return;

  for (size_t i = 0; i < size();) {
    if (solutions[rankingByCost[i]]->isClone) {
      Remove(i);
    } else {
      i++;
    }
  }
}

void ADCPopulation::Keep(size_t sz) {
  while (size() > sz) {
    size_t idxWorst = size() - 1;

    for (size_t i = 0; i < size() - 1; i++) {
      if (solutions[rankingByCost[i]]->fitness >= solutions[rankingByCost[idxWorst]]->fitness) {
        idxWorst = i;
      }
    }

    Remove(idxWorst, true);
  }
}

double ADCPopulation::AverageDC() const {
  double diversity = 0;

  size_t sz = size();
  for (size_t i = 0; i < sz; i++) {
    diversity += solutions[rankingByCost[i]]->dc;
  }

  if (size()) diversity /= size();

  return diversity;
}

double ADCPopulation::Distance(int s1, int s2) const {
  return distanceMatrix[s1][s2];
}

double ADCPopulation::AverageCost() const {
  double acc = 0.0;

  for (size_t i = 0; i < size(); i++) {
    acc += solutions[rankingByCost[i]]->Cost();
  }

  return acc / size();
}

bool ADCPopulation::Add(Solution* s) {
  Application::LogEvolution(s->Cost());

  ///////////////GET POSITION/////////////
  size_t curSize = size();
  size_t pos;
  for (pos = 0; pos < curSize; pos++) {
    if (s->Cost() < solutions[rankingByCost[pos]]->Cost()) break;
  }

  ////// INSERT AND MOVE SOLUTION RANKING //////
  s->idx = freeSolutions.top();
  freeSolutions.pop();
  solutions[s->idx] = s;
  for (; curSize > pos; curSize--) {
    rankingByCost[curSize] = rankingByCost[curSize - 1];
  }
  rankingByCost[pos] = s->idx;

  ////// UPDATE DISTANCE MATRIX /////
  double dist;
  curSize = size();
  s->isClone = false;
  for (int i = 0; i < nsz; i++) {
    if (solutions[i] != nullptr && solutions[i] != s) {
      dist = Application::instance->SolutionDistance(s, solutions[i]);
      distanceMatrix[s->idx][i] = distanceMatrix[i][s->idx] = dist;
      s->isClone |= dist < 0.01;
    } else {
      distanceMatrix[s->idx][i] = distanceMatrix[i][s->idx] = DBL_MAX;
    }
  }

  ///// UPDATE DIVERSITY CONTRIBUTION /////
  UpdateDiversityContribution(s);

  RecomputeFitness();

  return true;
}

void ADCPopulation::Remove(size_t pos, bool bRecompute) {
  size_t sz = size() - 1;

  Solution* s = solutions[rankingByCost[pos]];
  size_t idx = s->idx;

  // REMOVING FROM RANKING
  for (size_t i = pos; i < sz; i++)
    rankingByCost[i] = rankingByCost[i + 1];

  // UPDATE DISTANCE MATRIX
  size_t nclosest;
  bool updateAll = true;  // this->nclosest > sz-1;
  nclosest = std::min(this->nclosest, sz - 1);

  for (size_t i = 0; i < sz; i++) {
    double olddist = distanceMatrix[s->idx][rankingByCost[i]];

    distanceMatrix[s->idx][rankingByCost[i]] = distanceMatrix[rankingByCost[i]][s->idx] = DBL_MAX;

    if (updateAll || olddist <= maxDistance[rankingByCost[i]]) {
      CalculateDC(solutions[rankingByCost[i]], nclosest);
    }
  }

  delete solutions[idx];
  solutions[idx] = 0;
  freeSolutions.push(idx);
  maxDistance[idx] = DBL_MAX;
  if (bRecompute) RecomputeFitness();
}

void ADCPopulation::UpdateDiversityContribution(Solution* solution) {
  bool updateAll = nclosest > size() - 1;
  int nclosest = std::min(this->nclosest, size() - 1);

  CalculateDC(solution, nclosest);

  for (int i = 0; i < nsz; i++) {
    if ((solution->idx != i && solutions[i] != 0) &&                       // is valid
        (updateAll || maxDistance[i] > distanceMatrix[solution->idx][i]))  // is new distance
                                                                           // closer than max?
    {
      CalculateDC(solution, nclosest);
    }
  }
}

void ADCPopulation::CalculateDC(Solution* solution, int nclosest) {
  if (nclosest == 0) {
    maxDistance[solution->idx] = DBL_MAX;
    solution->dc = 1.0;
    return;
  }

  memcpy(distanceAux, distanceMatrix[solution->idx], nsz * sizeof(double));

  double distSum = 0;
  for (int i = 0; i < nclosest; i++) {
    int lessPos = i;
    for (int j = i + 1; j < nsz; j++) {
      if (distanceAux[j] < distanceAux[lessPos]) lessPos = j;
    }

    if (lessPos != i) std::swap(distanceAux[i], distanceAux[lessPos]);

    distSum += distanceAux[i];
  }

  maxDistance[solution->idx] = distanceAux[nclosest - 1];
  solution->dc = distSum / nclosest;
}

const Solution* ADCPopulation::BestSolution() const {
  return (*this)[0];
}

const Solution* ADCPopulation::BinaryTournament() const {
  size_t i = Random::RandomInt(0, size());
  size_t j = Random::RandomInt(0, size());
  const Solution* a = (*this)[(unsigned long)i];
  const Solution* b = (*this)[(unsigned long)j];

  return (a->fitness < b->fitness) ? a : b;
}

}  // namespace ga
