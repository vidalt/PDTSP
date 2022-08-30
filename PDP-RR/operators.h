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

#ifndef OPERATORS_H
#define OPERATORS_H

#include "instance.h"

class Operators {
  public:
    Operators();

    /**
     * @brief EvaluateRemoveRequest
     * @param instance
     * @param visits
     * @param pickuptIdx
     * @param deliveryIdx
     * @return
     */
    static double EvaluateRemoveRequest(const Instance& instance, std::vector<int>& visits, int pickuptIdx,
                                        int deliveryIdx, int* removePosition = nullptr);

    static double RemoveRequest(const Instance& instance, std::vector<int>& visits, int pickuptIdx,
                                int deliveryIdx);
    /**
     * @brief EvaluateBestInsertion
     * @param instance
     * @param visits
     * @param pickuptIdx
     * @param deliveryIdx
     * @param insertPosition
     * @return BestInsertionCost
     */
    static double EvaluateBestInsertion(const Instance& instance, std::vector<int>& visits, int pickuptIdx,
                                        int deliveryIdx, int insertPosition[]);

    /**
     * @brief EvaluateBestInsertionFast
     * @param instance
     * @param visits
     * @param pickuptIdx
     * @param deliveryIdx
     * @param insertPosition
     * @return BestInsertionCost
     */
    static double EvaluateBestInsertionFast(const Instance& instance, std::vector<int>& visits,
                                            int pickuptIdx, int deliveryIdx, int insertPosition[]);

    /**
     * @brief InsertPair
     * @param visits
     * @param pickuptIdx
     * @param deliveryIdx
     * @param insertPosition
     */
    static void InsertRequest(std::vector<int>& visits, int pickuptIdx, int deliveryIdx,
                              int insertPosition[]);

    typedef Instance::NodeList (*PtrRemoveOperator)(const Instance&, Solution&, int, double);

    static Instance::NodeList RandomRemoval(const Instance& instance, Solution& solution, int q, double p);
    static Instance::NodeList WorstRemoval(const Instance& instance, Solution& solution, int q, double p);
    static Instance::NodeList WorstDistanceRemoval(const Instance& instance, Solution& solution, int q,
                                                   double p);
    static Instance::NodeList WorstHandrlingRemoval(const Instance& instance, Solution& solution, int q,
                                                    double p);
    static Instance::NodeList BlockRemoval(const Instance& instance, Solution& solution, int q, double p);
};

#endif  // OPERATORS_H
