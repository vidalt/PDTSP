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

#ifndef PDPEDUCATE_H
#define PDPEDUCATE_H

#include "pdp/moves/pdp4optmove.h"
#include "pdp/moves/pdpmoveevaluation.h"
#include "pdp/pdpsolution.h"

class PDPNode;

namespace pdp {

class Educate : public std::vector<pdp::moves::PDPMove*> {
  public:
    //! Default constructor
    Educate();

    //! Educate destructor
    virtual ~Educate();

    //! Perform  Fast-Slow-Fast Local Search
    bool Run(PDPSolution* solution);

    //! Perform Fast Neighborhood Local Search
    bool FastNeighborhoods(PDPSolution* solution);

    //! Perform Slow Neighborhood Local Search
    bool SlowNeighborhoods(PDPSolution* solution);

    size_t Count() const {
      return educateCount;
    }
    size_t ResetCounts();

    std::string MovesLog(bool percent = false) const;
    std::string TotalMovesLog(bool percent = false) const;

  protected:
    //! Evaluate best neighborhood.
    //! \param solution: current Solution representation.
    //! \param pickupNode: pointer to Node that represents the pickup node to move.
    //! \return PDPMoveEvaluation: evaluation containing parameters for best move found in route.
    pdp::moves::PDPMoveEvaluation EvaluateBestNeighborhood(PDPSolution* solution, PDPNode* pickupNode);

    //! Evaluate best neighborhood.
    //! \param solution: current Solution representation.
    //! \return PDPMoveEvaluation: evaluation containing parameters for best move
    //! found in route.
    pdp::moves::PDPMoveEvaluation EvaluateBestNeighborhood(PDPSolution* solution);

  protected:
    //! pickup nodes to evaluate.
    std::vector<PDPNode*> pickupNodes;  // O(n/2) space

    size_t educateCount;
    size_t educateTotalCount;
};

}  // namespace pdp
#endif  // PDPEDUCATE_H
