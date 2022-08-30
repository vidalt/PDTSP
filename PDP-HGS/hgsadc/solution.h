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

#ifndef GA_SOLUTION_H
#define GA_SOLUTION_H

#include <iostream>

namespace ga {

class Solution {
  public:
    Solution(){};
    virtual ~Solution(){};
    virtual double Cost() const = 0;
    virtual Solution* Clone() const = 0;
    virtual void Copy(const Solution* s) = 0;

    //! Check if solution is feasible
    virtual bool IsFeasible() const = 0;

    //! Print solution state
    virtual void Print(std::ostream& os = std::cout) const = 0;

    bool operator<(const Solution& s) const {
      return this->Cost() < s.Cost();
    }

    Solution& operator=(const Solution& s) {
      this->Copy(&s);
      return *this;
    }

    int idx;
    bool isClone;

    double dc;
    double cost;
    double fitness;
};

}  // namespace ga

#endif  // GA_SOLUTION_H
