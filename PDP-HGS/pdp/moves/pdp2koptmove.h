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

#ifndef PDPk2optMove_H
#define PDPk2optMove_H

#include "pdp/moves/pdpmove.h"

namespace pdp {
namespace moves {

typedef struct _k2opmem {
    int I;
    int J;
    _k2opmem* next;
    double cost;
    _k2opmem() : _k2opmem(0, 0, 0, 0) {
    }

    _k2opmem(int I, int J, _k2opmem* next, double cost) {
      this->cost = std::min(cost, (double)DBL_MAX);
      this->I = I;
      this->J = J;
      this->next = next;
    }

    bool operator<(const _k2opmem& other) const {
      return this->cost < other.cost;
    }
} k2opmem;

class PDP2koptMove : public PDPMove {
  public:
    PDP2koptMove(bool allowInfeasible = false);
    virtual ~PDP2koptMove();

    //! Get local search move name.
    virtual std::string name() const;

    //! O(n) Evaluation of best move for a node to a given route.
    //! \param solution: current Solution representation.
    //! \return PDPMoveEvaluation: evaluation containing parameters for best move found in route.
    virtual PDPMoveEvaluation Evaluate(PDPSolution* solution);

    //! O(n) Evaluation of best move for a node to a given route.
    //! \param solution: current Solution representation.
    //! \param pickupNode: pointer to Node that represents the pickup node to move.
    //! \return PDPMoveEvaluation: evaluation containing parameters for best move found in route.
    virtual PDPMoveEvaluation Evaluate(PDPSolution* solution, PDPNode* pickupNode);

    //! Apply move with given parameters.
    //! \param solution: current Solution representation to be modified.
    //! \param eval: Previous evaluation with parameters for apply move.
    //! \return double: New Solution cost after local search move.
    virtual double move(PDPSolution* solution, const PDPMoveEvaluation& eval);

    virtual const char* ExtraTotalInfo() const;
    virtual const char* ExtraInfo() const;
    virtual size_t ResetCount();

  private:
    bool* visited;
    int* positions;
    k2opmem*** mem;
    size_t count2opt;
    size_t totalCount2opt;

    bool allowInfeasible;
};

}  // namespace moves
}  // namespace pdp

#endif  // PDPk2optMove_H
