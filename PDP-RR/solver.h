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

#ifndef SOLVER_H
#define SOLVER_H

#include <boost/program_options.hpp>
#include <vector>

#include "instance.h"

class Solver {
  public:
    Solver(boost::program_options::variables_map& variablesMap);
    virtual ~Solver();

    void PrintStats();
    void Solve();

  public:
    const Instance instance;
    double Ellapsed() const;

  protected:
    Solution solBest;

  private:
    typedef struct _EvolutionEntry {
        size_t iteration;
        double time;
        double cost;

        _EvolutionEntry() : _EvolutionEntry(0, 0, 0) {
        }
        _EvolutionEntry(size_t iteration_, double time_, double cost_)
            : iteration(iteration_), time(time_), cost(cost_) {
        }

    } EvolutionEntry;

    std::vector<EvolutionEntry> evolution;

    int qMin;
    int qMax;
    int iterationCount;

    double param_P;
    double param_C;
    double param_MaxIt;
    bool param_Fast;
};

#endif  // SOLVER_H
