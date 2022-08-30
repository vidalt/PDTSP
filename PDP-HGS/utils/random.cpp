#include "random.h"

int Random::RandomInt() {
  return std::rand();
}

int Random::RandomInt(int a, int b) {
  return a + RandomReal() * (b - a);
}

double Random::RandomReal() {
  return std::rand() / static_cast<double>(RAND_MAX);
}

double Random::RandomReal(double a, double b) {
  return a + RandomReal() * (b - a);
}
