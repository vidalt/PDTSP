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

#ifndef PDPMOVEEVALUATION_H
#define PDPMOVEEVALUATION_H

#include <float.h>

#include <string>

#include "pdp/pdpsolution.h"

namespace pdp {
namespace moves {

class PDPMove;
class PDPMoveEvaluation {
  public:
    //!  Default constructor
    PDPMoveEvaluation() {
      cost = DBL_MAX;
      neighborhood = 0;
    }

    //!  Apply current move
    //!  \param solution: solution representation to apply move
    //!  \return Returns true if move applied succesfully
    bool Apply(PDPSolution* solution, bool force = false) const;

    //!  Expected cost after move
    double cost;

    //!  parameters to execute the move
    void* moveparam;

    //!  Selected neighborhood
    PDPMove* neighborhood;
};

}  // namespace moves
}  // namespace pdp

#endif  // PDPMOVEEVALUATION_H
