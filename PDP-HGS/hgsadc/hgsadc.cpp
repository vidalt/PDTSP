#include "hgsadc.h"

#include <algorithm>
#include <boost/bind/bind.hpp>

#include "hgsadc/adcpopulation.h"
#include "utils/application.h"
#include "utils/random.h"

using namespace std;
#include <iomanip>

namespace ga {

HGSADC::HGSADC() {
}
int countAll = 0;
double accDistance = 0;
double distToChild = 0;

void HGSADC::SelectParents(const ADCPopulation& population, Solution*& p1, Solution*& p2) {
  bool singleSolution = population.size() == 1;
  p1 = (Solution*)population.BinaryTournament();
  do {
    p2 = (Solution*)population.BinaryTournament();
  } while (p1 == p2 && !singleSolution);
}

void HGSADC::DiversifyPopulation(ADCPopulation& population, const int numberOfIndividuals) {
  population.Keep(Application::hgsadc_populationSize / 3);

  for (int i = 0; i < numberOfIndividuals; i++) {
    Solution* s = Application::instance->CreateRandomSolution();

    Solution* s2 = s->Clone();
    Application::instance->Mutate(s2);
    Application::instance->Repair(s2);
    Application::instance->Educate(s2);
    if (*s2 < *s) *s = *s2;
    delete s2;

    population.Add(s);

    if (Application::Timeout()) break;
  }

  SelectSurvivors(population);
}

void HGSADC::InitializePopulation(ADCPopulation& population, const int numberOfIndividuals) {
  population.Keep(0);
  for (int i = 0; i < numberOfIndividuals; i++) {
    Solution* s = Application::instance->CreateRandomSolution();

    Solution* s2 = s->Clone();
    Application::instance->Mutate(s2);
    Application::instance->Repair(s2);
    Application::instance->Educate(s2);
    if (*s2 < *s) *s = *s2;
    delete s2;

    population.Add(s);

    if (Application::Timeout()) break;
  }

  SelectSurvivors(population);
}

void HGSADC::SelectSurvivors(ADCPopulation& population) {
  population.RemoveClones();
  population.Keep(Application::hgsadc_populationSize);
}

void PrintGenerationLog(Problem& problem, Solution* best, ADCPopulation& population, int generationCount,
                        int itWithoutImprovement, bool bComplete = false) {
  if (Application::verbose) {
    cout << "\tGeneration: " << generationCount << ", Time: " << Application::Ellapsed()
         << ", Best: " << std::fixed << setprecision(0) << best->Cost() << ", Diversity: " << std::fixed
         << setprecision(3) << population.AverageDC() << ", IWI: " << itWithoutImprovement
         << ", AVG: " << population.AverageCost() << endl;
    problem.LSLogReset();

    if (bComplete) {
      for (size_t i = 0; i < population.size(); i++) {
        cout << endl << endl << setprecision(0) << "\t=> Sol #" << i + 1 << endl;
        cout << "\tDC: " << population[i]->dc << endl;
        population[i]->Print();
      }
    }
  }
}

void HGSADC::Solve(Solution* best, Problem& problem) {
  int iterationsCount = 0;
  int generationCount = 1;
  int iterationsWithoutImprovement = 0;
  int diversifyCount = Application::hgsadc_divIterationsWithoutImprovement;
  std::ios oldState(nullptr);
  oldState.copyfmt(std::cout);

  ADCPopulation population;

  if (Application::verbose) {
    std::cout << "\t=> METHOD: HGSADC" << endl;
  }

  InitializePopulation(population, Application::hgsadc_populationSize * 4);
  *best = *population.BestSolution();

  while (iterationsWithoutImprovement < Application::hgsadc_maxIterationsWithoutImprovement &&
         !Application::Timeout()) {
    if (iterationsCount % Application::hgsadc_offspringInGeneration == 0) {
      PrintGenerationLog(problem, best, population, generationCount, iterationsWithoutImprovement, false);
    }

    iterationsCount++;

    // OFFSPRING GENERATION
    Solution* p1;
    Solution* p2;
    SelectParents(population, p1, p2);
    Solution* child = Application::instance->CreateEmptySolution();

    problem.Crossover(child, p1, p2);
    problem.Mutate(child);
    problem.Repair(child);
    problem.Educate(child);

    // UPDATE POPULATION
    bool updateBest = *child < *best;
    population.Add(child);
    if ((iterationsCount % Application::hgsadc_offspringInGeneration) == 0) {
      SelectSurvivors(population);
      generationCount++;
    }

    if (updateBest) {
      *best = *child;
      iterationsWithoutImprovement = 0;
    } else {
      iterationsWithoutImprovement++;
      if (iterationsWithoutImprovement % diversifyCount == 0) {
        if (Application::verbose) cout << "\t=> Diversifying..." << endl;

        DiversifyPopulation(population, Application::hgsadc_populationSize * 4);

        if (*population.BestSolution() < *best) {
          *best = *population.BestSolution();
          iterationsWithoutImprovement = 0;
        }
      }
    }
  }

  // FINAL INFORMATIONS
  SelectSurvivors(population);
  PrintGenerationLog(problem, best, population, generationCount, iterationsWithoutImprovement, true);

  std::cout.copyfmt(oldState);
}

}  // namespace ga
