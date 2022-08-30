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

#ifndef PDPSOLUTION_H
#define PDPSOLUTION_H
#include <limits.h>

#include <iostream>
#include <vector>

#include "hgsadc/solution.h"
#include "pdp/pdproute.h"

class Instance;

namespace pdp {

class PDPSolution : public ga::Solution {
  public:
    //! Structure to represent solution distance.
    struct SolutionDistance {
        //! Distance to other solution.
        int distance;

        //! Index of distance relative solution.
        int to;

        //! Default Constructor.
        SolutionDistance() {
          distance = INT_MAX;
        }

        //! less than operator.
        bool operator<(const SolutionDistance& s) const {
          return distance < s.distance;
        }
    };

  public:
    //! Copy constructor
    PDPSolution(const PDPSolution&);

    //! Default constructor
    PDPSolution();

    //! Destructor
    virtual ~PDPSolution();

    virtual ga::Solution* Clone() const;
    virtual void Copy(const ga::Solution* s);

    //! Compute routes information.
    void Recompute();

    //! Compute all routes positions
    void ComputePositions();

    //! Compute specific route position
    //! \param routeIdx: route index to compute customer positions.
    void ComputePosition(int routeIdx);

    //! Check if solution is feasible
    virtual bool IsFeasible() const;

    //! Print solution state
    virtual void Print(std::ostream& os = std::cout) const;

    //! Get customer postion
    //! \param k: id of customer
    //! \return int: route and in-route index of the given customer.
    int FindPosition(int k) const;

    //! Assignment operator
    const PDPSolution& operator=(const PDPSolution&);

    virtual double Cost() const;

  public:
    pdp::PDPRoute route;

  private:
    std::vector<int> positions;
};

}  // namespace pdp
#endif  // PDPSOLUTION_H
