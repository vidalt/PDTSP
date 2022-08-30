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

#ifndef PDPMOVE_H
#define PDPMOVE_H

#include <cstring>
#include <string>

#include "pdp/moves/pdpmoveevaluation.h"
#include "pdp/pdpnode.h"

namespace pdp {
namespace moves {

class PDPMove {
  public:
    //! PDPMove constructor.
    PDPMove() {
      totalCount = 0;
      count = 0;
      cpuTime = 0.0;
    }

    //! PDPMove destructor.
    virtual ~PDPMove() {
    }

    //! Get local search move name.
    virtual std::string name() const = 0;

    //! Evaluate best move for a node to a given route.
    //! \param solution: current Solution representation.
    //! \param pickupNode: pointer to Node that represents the pickup node to move.
    //! \return MoveEvaluation: evaluation containing parameters for best move found in route.
    virtual PDPMoveEvaluation Evaluate(PDPSolution* solution, PDPNode* pickupNode) = 0;

    //! Evaluate best move to a given route.
    //! \param solution: current Solution representation.
    //! \return MoveEvaluation: evaluation containing parameters for best move found in route.
    virtual PDPMoveEvaluation Evaluate(PDPSolution* solution) = 0;

    //! Apply move with given parameters.
    //! \param solution: current Solution representation to be modified.
    //! \param eval: Previous evaluation with parameters for apply move.
    //! \return double: New Solution cost after local search move.
    virtual double move(PDPSolution* solution, const PDPMoveEvaluation& eval) {
      count++;
      totalCount++;

      solution->ComputePositions();
      return solution->Cost();
    }

    virtual const char* ExtraTotalInfo() const {
      return 0;
    }

    virtual const char* ExtraInfo() const {
      return 0;
    }

    size_t TotalCount() const {
      return totalCount;
    }

    size_t Count() const {
      return count;
    }

    virtual size_t ResetCount() {
      size_t ret = count;
      count = 0;
      return ret;
    }

    void AddCpuTime(double cpuTime) {
      this->cpuTime += cpuTime;
    }
    double CpuTime() const {
      return cpuTime;
    }

  private:
    double cpuTime;
    size_t count;
    size_t totalCount;

  protected:
    char extraInfoBuffer[256];
};

}  // namespace moves
}  // namespace pdp

#endif  // PDPMOVE_H
