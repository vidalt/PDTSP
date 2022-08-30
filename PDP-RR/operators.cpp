#include "operators.h"

#include <float.h>

#include <algorithm>
#include <set>

#include "random.h"

Operators::Operators() {
}

double Operators::RemoveRequest(const Instance& instance, std::vector<int>& visits, int pickuptIdx,
                                int deliveryIdx) {
  double cost = 0.0;
  int pickupPos, deliveryPos;

  for (size_t i = 0; i < visits.size(); i++) {
    if (visits[i] == pickuptIdx)
      pickupPos = i;
    else if (visits[i] == deliveryIdx)
      deliveryPos = i;
  }

  // rm delivery
  cost += instance.distances[visits[deliveryPos - 1]][visits[deliveryPos + 1]] -
          instance.distances[visits[deliveryPos - 1]][deliveryIdx] -
          instance.distances[deliveryIdx][visits[deliveryPos + 1]];
  visits.erase(visits.begin() + deliveryPos);

  // rm pickup
  cost += instance.distances[visits[pickupPos - 1]][visits[pickupPos + 1]] -
          instance.distances[visits[pickupPos - 1]][pickuptIdx] -
          instance.distances[pickuptIdx][visits[pickupPos + 1]];
  visits.erase(visits.begin() + pickupPos);

  return cost;
}

double Operators::EvaluateRemoveRequest(const Instance& instance, std::vector<int>& visits, int pickuptIdx,
                                        int deliveryIdx, int* removePosition) {
  double cost = 0.0;
  int pickupPos, deliveryPos;

  for (size_t i = 0; i < visits.size(); i++) {
    if (visits[i] == pickuptIdx)
      pickupPos = i;
    else if (visits[i] == deliveryIdx)
      deliveryPos = i;
  }

  if (deliveryPos - pickupPos == 1) {
    cost = instance.distances[visits[pickupPos - 1]][visits[deliveryPos + 1]] -
           instance.distances[visits[pickupPos - 1]][pickuptIdx] -
           instance.distances[pickuptIdx][deliveryIdx] -
           instance.distances[deliveryIdx][visits[deliveryPos + 1]];
  } else {
    // rm delivery
    cost += instance.distances[visits[deliveryPos - 1]][visits[deliveryPos + 1]] -
            instance.distances[visits[deliveryPos - 1]][deliveryIdx] -
            instance.distances[deliveryIdx][visits[deliveryPos + 1]];

    // rm pickup
    cost += instance.distances[visits[pickupPos - 1]][visits[pickupPos + 1]] -
            instance.distances[visits[pickupPos - 1]][pickuptIdx] -
            instance.distances[pickuptIdx][visits[pickupPos + 1]];
  }

  if (removePosition != nullptr) {
    removePosition[0] = pickupPos;
    removePosition[1] = deliveryPos;
  }

  return cost;
}

void Operators::InsertRequest(std::vector<int>& visits, int pickuptIdx, int deliveryIdx,
                              int insertPosition[]) {
  visits.insert(visits.begin() + insertPosition[1], deliveryIdx);
  visits.insert(visits.begin() + insertPosition[0], pickuptIdx);
}

double Operators::EvaluateBestInsertionFast(const Instance& instance, std::vector<int>& visits,
                                            int pickuptIdx, int deliveryIdx, int insertPosition[]) {
  double** distances = instance.distances;
  double bestCost = DBL_MAX;

  // considering non-sequencial insertion
  int bestDelivery = -1;
  double bestDeliveryCost = DBL_MAX;
  for (size_t i = visits.size() - 2; i > 0; i--) {
    double pickupCost = -distances[visits[i - 1]][visits[i]] + distances[visits[i - 1]][pickuptIdx] +
                        distances[pickuptIdx][visits[i]];

    double deliveryCost = -distances[visits[i]][visits[i + 1]] + distances[visits[i]][deliveryIdx] +
                          distances[deliveryIdx][visits[i + 1]];

    if (deliveryCost <= bestDeliveryCost) {
      bestDeliveryCost = deliveryCost;
      bestDelivery = i + 1;
    }

    double cost = pickupCost + bestDeliveryCost;

    if (cost <= bestCost) {
      bestCost = cost;
      insertPosition[0] = i;
      insertPosition[1] = bestDelivery;
    }
  }

  // considering sequencial insertion
  for (size_t p = 1; p < visits.size(); p++) {
    double cost = instance.distances[visits[p - 1]][pickuptIdx] +
                  instance.distances[pickuptIdx][deliveryIdx] + instance.distances[deliveryIdx][visits[p]] -
                  instance.distances[visits[p - 1]][visits[p]];

    if (cost <= bestCost) {
      bestCost = cost;
      insertPosition[0] = p;
      insertPosition[1] = p;
    }
  }

  return bestCost;
}

double Operators::EvaluateBestInsertion(const Instance& instance, std::vector<int>& visits, int pickuptIdx,
                                        int deliveryIdx, int insertPosition[]) {
  double bestCost = DBL_MAX;

  // considering non-sequencial insertion
  for (size_t p = 1; p < visits.size() - 1; p++) {
    double pCost = +instance.distances[visits[p - 1]][pickuptIdx] +
                   instance.distances[pickuptIdx][visits[p]] - instance.distances[visits[p - 1]][visits[p]];

    for (size_t d = p + 1; d < visits.size(); d++) {
      double cost = pCost + instance.distances[visits[d - 1]][deliveryIdx] +
                    instance.distances[deliveryIdx][visits[d]] - instance.distances[visits[d - 1]][visits[d]];

      if (cost < bestCost) {
        bestCost = cost;
        insertPosition[0] = p;
        insertPosition[1] = d;
      }
    }
  }

  // considering sequencial insertion
  for (size_t p = 1; p < visits.size(); p++) {
    double cost = instance.distances[visits[p - 1]][pickuptIdx] +
                  instance.distances[pickuptIdx][deliveryIdx] + instance.distances[deliveryIdx][visits[p]] -
                  instance.distances[visits[p - 1]][visits[p]];

    if (cost <= bestCost) {
      bestCost = cost;
      insertPosition[0] = p;
      insertPosition[1] = p;
    }
  }

  return bestCost;
}

Instance::NodeList Operators::RandomRemoval(const Instance& instance, Solution& sol, int q, double p) {
  std::set<int> selecteds;
  std::set<int>::iterator it;
  Instance::NodeList removedRequests;
  for (int i = 0; i < q; i++) {
    int rem;

    do {
      rem = sol.visits[Random::RandomInt(1, sol.visits.size() - 1)];
      if (instance.nodes[rem]->isDelivery) rem = instance.nodes[rem]->pair;

      it = selecteds.find(rem);
    } while (it != selecteds.end());
    selecteds.insert(rem);

    removedRequests.push_back(instance.nodes[rem]);
  }

  return removedRequests;
}

Instance::NodeList Operators::WorstRemoval(const Instance& instance, Solution& sol, int q, double p) {
  Instance::NodeList pickups = instance.Pickups();
  std::vector<double> pickupsOrder;
  std::vector<double> removalCosts;
  for (size_t i = 0; i < pickups.size(); i++) {
    pickupsOrder.push_back(i);

    const Instance::Node* node = pickups[i];
    removalCosts.push_back(Operators::EvaluateRemoveRequest(instance, sol.visits, node->idx, node->pair));
  }

  std::sort(pickupsOrder.begin(), pickupsOrder.end(),
            [&, removalCosts](int a, int b) { return removalCosts[a] < removalCosts[b]; });

  Instance::NodeList removedRequests;
  for (int i = 0; i < q; i++) {
    int r = std::pow(Random::RandomReal(), p) * pickupsOrder.size();
    removedRequests.push_back(pickups[pickupsOrder[r]]);
    pickupsOrder.erase(pickupsOrder.begin() + r);
  }

  return removedRequests;
}

Instance::NodeList Operators::WorstDistanceRemoval(const Instance& instance, Solution& sol, int q, double p) {
  return WorstRemoval(instance, sol, q, p);
}

Instance::NodeList Operators::WorstHandrlingRemoval(const Instance& instance, Solution& sol, int q,
                                                    double p) {
  return WorstRemoval(instance, sol, q, p);
}

Instance::NodeList Operators::BlockRemoval(const Instance& instance, Solution& sol, int q, double p) {
  int k = sol.visits[Random::RandomInt(1, sol.visits.size() - 1)];
  const Instance::Node* pickup = instance.nodes[k];
  if (pickup->isDelivery) pickup = instance.nodes[pickup->pair];

  int i;
  std::set<int> selecteds;
  Instance::NodeList removedRequests;

  for (i = 1; sol.visits[i] != pickup->idx; i++)
    ;

  for (; removedRequests.size() < q && sol.visits[i] != pickup->pair; i++) {
    const Instance::Node* request = instance.nodes[sol.visits[i]];
    if (request->isDelivery) request = instance.nodes[request->pair];

    if (selecteds.find(request->idx) != selecteds.end()) continue;

    selecteds.insert(request->idx);
    removedRequests.push_back(request);
  }

  return removedRequests;
}
