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

#ifndef HGSADC_H
#define HGSADC_H

#include "hgsadc/adcpopulation.h"
#include "hgsadc/problem.h"
#include "hgsadc/solution.h"

namespace ga {

class HGSADC {
  private:
    HGSADC();

  public:
    static void Solve(Solution* s, Problem& problem);

  protected:
    static void SelectParents(const ADCPopulation& population, Solution*& p1, Solution*& p2);

    static void InitializePopulation(ADCPopulation& population, const int numberOfIndividuals);

    static void DiversifyPopulation(ADCPopulation& population, const int numberOfIndividuals);

    static void SelectSurvivors(ADCPopulation& population);
};

}  // namespace ga

#endif  // HGSADC_H
