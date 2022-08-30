#include "pdproute.h"

#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>

#include "utils/application.h"

using namespace std;

namespace pdp {

PDPRoute::~PDPRoute() {
}

PDPRoute::PDPRoute() {
}

double PDPRoute::Cost(const PDPRouteInfo& info) {
  if (info.size == 0) return 0;

  return info.distance;
}

PDPRoute* PDPRoute::clone() const {
  return new PDPRoute(*this);
}

PDPRoute::PDPRoute(const PDPRoute& r) {
  *this = r;
}

const PDPRoute& PDPRoute::operator=(const PDPRoute& r) {
  std::vector<int>::operator=(r);
  feasible = r.feasible;
  cost = r.cost;
  return *this;
}

double PDPRoute::Cost() const {
  return cost;
}

void PDPRoute::Print(std::ostream& os) const {
  os << "[0";
  for (const_iterator customer = begin() + 1; customer != end() - 1; customer++)
    os << ", " << *customer;
  os << ", 0]";
}

double PDPRoute::PrecomputeRouteInformation() {
  cost = 0;
  double** distances = Application::instance->Distances();
  for (size_t i = 0; i < size() - 1; i++) {
    cost += distances[at(i)][at(i + 1)];
  }
  return Cost();
}

bool PDPRoute::isEmpty() const {
  return size() < 3;
}

}  // namespace pdp
