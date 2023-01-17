#include "solver.h"

#include <float.h>

#include <iostream>
#include <string>

#include "application.h"
#include "operators.h"
#include "random.h"

using namespace std;

Solver::Solver(boost::program_options::variables_map& variablesMap)
    : instance(variablesMap["instance"].as<string>()) {
  int n = instance.nodes.size() / 2;
  qMin = std::min(30, int(0.20 * n));
  qMax = std::min(50, int(0.55 * n));

  param_P = variablesMap["p-accept"].as<double>();
  param_C = variablesMap["c-rate"].as<double>();
  param_MaxIt = variablesMap["it"].as<int>();
  param_Fast = variablesMap.count("fast") > 0;

  if (variablesMap.count("time-limit") > 0) {
    Application::SetTimeLimit(variablesMap["time-limit"].as<int>());
  }
}

Solver::~Solver() {
}

/**
 * @brief Create initial solution based on Ropke and Pisinger (2006):
 * Ropke, S. , & Pisinger, D. (2006). An adaptive large neighborhood search
 * heuristic for the pickup and delivery problem with time windows.
 * Transportation Science, 40 (4), 455–472
 * @param instance
 * @param useFast
 * @return
 */
Solution InitialSolution(const Instance& instance, bool useFast) {
  Solution sol;
  sol.visits.push_back(0);
  sol.visits.push_back(0);

  Instance::NodeList pickupNodes = instance.Pickups();

  const auto bestInsertionMethod =
      useFast ? Operators::EvaluateBestInsertionFast : Operators::EvaluateBestInsertion;

  while (pickupNodes.size()) {
    int bestPos = -1;
    int bestInsertionPosition[2] = {-1, -1};
    double bestInsertionCost = DBL_MAX;

    for (size_t pos = 0; pos < pickupNodes.size(); pos++) {
      const Instance::Node* pickup = pickupNodes[pos];

      int insertPosition[2];
      double insertCost =
          bestInsertionMethod(instance, sol.visits, pickup->idx, pickup->pair, insertPosition);
      if (insertCost < bestInsertionCost) {
        bestPos = pos;
        bestInsertionCost = insertCost;
        memcpy(bestInsertionPosition, insertPosition, sizeof(int) * 2);
      }
    }

    sol.cost += bestInsertionCost;
    Operators::InsertRequest(sol.visits, pickupNodes[bestPos]->idx, pickupNodes[bestPos]->pair,
                             bestInsertionPosition);

    pickupNodes.erase(pickupNodes.begin() + bestPos);
  }

  return sol;
}

Instance::NodeList RemoveRequests(const Instance& instance, Solution& sol, int q, int p) {
  static std::vector<Operators::PtrRemoveOperator> removalOperators = {
      Operators::RandomRemoval, Operators::WorstDistanceRemoval, Operators::BlockRemoval};

  Operators::PtrRemoveOperator removalOp = removalOperators[Random::RandomInt(0, removalOperators.size())];
  Instance::NodeList removedRequests = removalOp(instance, sol, q, p);

  for (const Instance::Node* node : removedRequests) {
    sol.cost += Operators::RemoveRequest(instance, sol.visits, node->idx, node->pair);
  }

  return removedRequests;
}

void ReinsertRequests(const Instance& instance, Solution& sol, Instance::NodeList removedRequests,
                      bool useFast) {
  const auto bestInsertionMethod =
      useFast ? Operators::EvaluateBestInsertionFast : Operators::EvaluateBestInsertion;

  Random::shuffle(removedRequests.begin(), removedRequests.end());

  for (const Instance::Node* node : removedRequests) {
    int insertPosition[2];
    double insertCost = bestInsertionMethod(instance, sol.visits, node->idx, node->pair, insertPosition);
    sol.cost += insertCost;
    Operators::InsertRequest(sol.visits, node->idx, node->pair, insertPosition);
  }
}

void Solver::Solve() {
  iterationCount = 0;
  Solution sol, solPrime;
  Instance::NodeList removedRequests;

  sol = InitialSolution(instance, param_Fast);
  evolution.push_back(EvolutionEntry(iterationCount, Application::Ellapsed(), sol.cost));

  solBest = sol;
  double temperature;
  double tempStart = -0.05 * solBest.cost / std::log(0.5);

  while ((iterationCount < param_MaxIt || Application::UsingTimeLimit()) && !Application::Timeout()) {
    iterationCount++;

    solPrime = sol;

    int q = Random::RandomInt(qMin, qMax);
    removedRequests = RemoveRequests(instance, solPrime, q, param_P);
    ReinsertRequests(instance, solPrime, removedRequests, param_Fast);

    temperature = tempStart;
    temperature *= Application::UsingTimeLimit()
                       ? std::pow(param_C, param_MaxIt * Application::EllapsedRatio())
                       : std::pow(param_C, static_cast<double>(iterationCount));

    double acceptProbability = std::exp(-(solPrime.cost - sol.cost) / temperature);

    if (Random::RandomReal() < acceptProbability) {
      sol = solPrime;
    }

    if (solPrime < solBest) {
      solBest = solPrime;
      evolution.push_back(EvolutionEntry(iterationCount, Application::Ellapsed(), sol.cost));

      if (Application::IsVerbose()) {
        std::cout << "New solution found: " << solBest.cost << "   IT: " << iterationCount << std::endl;
      }
    }
  }
}

void Solver::PrintStats() {
  double cost = 0.0;

  for (size_t i = 0; i < solBest.visits.size() - 1; i++) {
    cost += instance.distances[solBest.visits[i]][solBest.visits[i + 1]];
  }

  if (Application::IsVerbose()) {
    std::cout << std::endl;
    std::cout << iterationCount;
    std::cout << "; " << Application::Ellapsed();
    std::cout << "; " << solBest.cost << std::endl;
  } else {
    std::cout << "{" << std::endl;
    std::cout << "  \"version\": \"" << Application::Version() << "\"";
    std::cout << "," << std::endl << "  \"cost\": " << solBest.cost;
    std::cout << "," << std::endl << "  \"time\": " << Application::Ellapsed();
    std::cout << "," << std::endl << "  \"educate\": " << iterationCount;
    std::cout << "," << std::endl << "  \"solution\": [0";
    for (int v = 1; v < solBest.visits.size(); v++) {
      std::cout << ", " << solBest.visits[v];
    }

    std::cout << "]," << std::endl << "  \"evolution\": [" << std::endl;
    std::vector<EvolutionEntry>::iterator evolIt = evolution.begin();
    for (size_t i = 0; i < evolution.size() - 1; i++) {
      EvolutionEntry& entry = *evolIt;
      std::cout << "    {\n";
      std::cout << "       \"iteration\": " << entry.iteration << ",\n";
      std::cout << "       \"time\": " << entry.time << ",\n";
      std::cout << "       \"cost\": " << entry.cost << "\n";
      std::cout << "    },\n";
      evolIt++;
    }
    EvolutionEntry& entry = *evolIt;
    std::cout << "    {\n";
    std::cout << "       \"iteration\": " << entry.iteration << ",\n";
    std::cout << "       \"time\": " << entry.time << ",\n";
    std::cout << "       \"cost\": " << entry.cost << "\n";
    std::cout << "    }\n";
    std::cout << "  ]";

    std::cout << std::endl << "}" << std::endl;
  }
}

using namespace std;
int main(int argc, char* argv[]) {
  boost::program_options::variables_map variablesMap = Application::initializeVariablesMap(argc, argv);

  unsigned int seed = (unsigned int)variablesMap["seed"].as<int>();
  std::srand(seed);

  if (variablesMap["instance"].empty())  // solve only one instance specified by
                                         // the --instance cmdline argument
  {
    cout << "No input instance. Use --help for more information" << endl;
    exit(1);
  }

  // Create solver
  Solver solver(variablesMap);

  // Run
  solver.Solve();

  // Print
  solver.PrintStats();

  return 0;
}
