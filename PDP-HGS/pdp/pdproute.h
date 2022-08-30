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

#ifndef PDPROUTE_H
#define PDPROUTE_H

#include <stdarg.h>

#include <iostream>
#include <ostream>
#include <vector>

#include "pdp/pdprouteinfo.h"

namespace pdp {

class PDPRoute : public std::vector<int> {
  public:
    //! Default constructor
    PDPRoute();

    //! Copy constructor
    PDPRoute(const PDPRoute&);

    //! Destructor
    virtual ~PDPRoute();

    //! Create an identical clone of route.
    virtual PDPRoute* clone() const;

    //! Check if route had only depots.
    virtual bool isEmpty() const;

    //! Get penalized cost.
    virtual double Cost() const;

    //! Compute cost of subroute
    //! \param info: Subroute info to calculate cost.
    //! \return double: Penalized cost of subroute.
    static double Cost(const PDPRouteInfo& info);

    //! Assignment operator
    const PDPRoute& operator=(const PDPRoute&);

    //! Prints route state.
    virtual void Print(std::ostream& os = std::cout) const;

    //! Precompute helper struture for faster neighborhood evaluation.
    virtual double PrecomputeRouteInformation();

  protected:
    int feasible;
    double cost;
};

}  // namespace pdp

#endif  // PDPROUTE_H
