/*MIT License
 *
Copyright(c) 2020 Thibaut Vidal

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

#ifndef PROBLEM_H
#define PROBLEM_H

#include <string>

#include "hgsadc/solution.h"

namespace ga {
class Problem {
  public:
    virtual ~Problem(){};

    //! Genetic Algorithm Crossover for pickup and delivery
    //! \param child:  solution to be filled with a random valid crossover solution.
    //! \param p1: const reference for first parent to be crossed.
    //! \param p2: const reference for seccond parent to be crossed.
    //! \return resulting crossover child.
    virtual void Crossover(Solution* child, const Solution* p1, const Solution* p2) = 0;

    //! Repair solution
    //! \param solution: Solution to be repaired
    //! \param trace: if true, every repair step will be displayed.
    virtual void Repair(Solution* s) = 0;

    //! Mutate solution.
    virtual void Mutate(Solution* s) = 0;

    //! Perform local search
    //! \param solution: solution representation to be changed by local search
    //! \param trace: if true, every local search step will be displayed.
    virtual void Educate(Solution* s) = 0;

    virtual double SolutionDistance(const ga::Solution* a, const ga::Solution* b) const = 0;

    virtual void* Data() = 0;
    virtual void Precompute() = 0;
    virtual size_t Size() const = 0;

    virtual ga::Solution* CreateRandomSolution() const = 0;
    virtual ga::Solution* CreateEmptySolution() const = 0;

    virtual std::string LSCompleteLog() = 0;
    virtual std::string LSLog() = 0;
    virtual void LSLogReset() = 0;

    inline double**& Distances() {
      return distances;
    }

  protected:
    double** distances;
};
};  // namespace ga

#endif  // PROBLEM_H
