/*MIT License
 *
Copyright(c) 2022 Toni Pacheco

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#ifndef ADCPOPULATION_H
#define ADCPOPULATION_H

#include <stddef.h>

#include <stack>

#include "hgsadc/solution.h"

namespace ga {
class ADCPopulation {
  public:
    ADCPopulation();
    virtual ~ADCPopulation();

    bool Add(Solution* s);
    void Keep(size_t sz);
    void Remove(size_t rankingPos, bool bRecompute = true);
    void RemoveClones();

    size_t size() const;
    const Solution* operator[](size_t p) const;

    const Solution* BestSolution() const;
    const Solution* BinaryTournament() const;

    void RecomputeFitness();

    double AverageCost() const;
    double AverageDC() const;

  private:
    inline Solution* at(size_t pos) const;
    double Distance(int s1, int s2) const;
    double DiversityContribution(int idx) const;

  private:
    int nsz;
    Solution** solutions;
    std::stack<int> freeSolutions;

    double* maxDistance;

    size_t populationSize;
    size_t nclosest;

    int* rankingByCost;
    int* rankingByDiversity;
    int* rankingByDiversityAux;

    double** distanceMatrix;
    double* distanceAux;

    void UpdateDiversityContribution(Solution* solution);

    void CalculateDC(Solution* solution, int closest);
};

}  // namespace ga

#endif  // ADCPOPULATION_H
