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

#ifndef PDP2optMove_H
#define PDP2optMove_H

#include "pdp/moves/pdpmove.h"

namespace pdp {
namespace moves {

class PDP2optMove : public PDPMove {
    typedef struct _PDP2optMoveState {
        int originPickup;
        int originDelivery;
        int destinyPickup;
        int destinyDelivery;
    } PDP2optMoveState;

  public:
    PDP2optMove();
    virtual ~PDP2optMove();

    //! Get local search move name.
    virtual std::string name() const;

    //! O(n) Evaluation of best move for a node to a given route.
    //! \param solution: current Solution representation.
    //! \param pickupNode: pointer to Node that represents the pickup node to move.
    //! \return PDPMoveEvaluation: evaluation containing parameters for best move found in route.
    virtual PDPMoveEvaluation Evaluate(PDPSolution* solution, PDPNode* pickupNode);

    //! O(n) Evaluation of best move for a given route.
    //! \param solution: current Solution representation.
    //! \param pickupNode: pointer to Node that represents the pickup node to move.
    //! \return PDPMoveEvaluation: evaluation containing parameters for best move found in route.
    virtual PDPMoveEvaluation Evaluate(PDPSolution* solution);

    //! Apply move with given parameters.
    //! \param solution: current Solution representation to be modified.
    //! \param eval: Previous evaluation with parameters for apply move.
    //! \return double: New Solution cost after local search move.
    virtual double move(PDPSolution* solution, const PDPMoveEvaluation& eval);

  private:
    bool* visited;
    PDP2optMoveState state;
};

}  // namespace moves
}  // namespace pdp

#endif  // PDP2optMove_H
