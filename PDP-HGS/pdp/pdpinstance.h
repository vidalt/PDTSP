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

#ifndef PDPInstance_H
#define PDPInstance_H

#include <set>
#include <string>
#include <vector>

#include "hgsadc/problem.h"
#include "pdp/moves/pdprelocatemove.h"
#include "pdp/pdpeducate.h"

namespace pdp {

class PDPInstance : public ga::Problem {
  public:
    //! Constructor
    //! \param vehicles: Maximum number of vehicles (or routes)
    //! \param capacity: Vehicle maximum capacity.
    //! \param maxDuration: Route maximum duration.
    //! \param numberOfCloseIndividuals: Number of closest individuals relevant in
    //! local search \param nodes: Customer nodes. \param comment: Instance
    //! comments \param vehicleSpeed: Vehicle speed.
    PDPInstance(int numberOfCloseIndividuals, const NodeList& nodes, const std::string comment = "");

    //! Create an instance from file
    //! \param instanceFilePath: Path for instance file instance
    static PDPInstance* fromFilePath(const std::string instanceFilePath);

    //! Destructor
    virtual ~PDPInstance();

    //! Precompute post-load informations
    virtual void Precompute();

    //! Perform local search
    //! \param solution: solution representation to be changed by local search
    //! \param trace: if true, every local search step will be displayed.
    virtual void Educate(ga::Solution* solution);

    //! \param s: solution to be filled with a random valid solution
    virtual ga::Solution* CreateEmptySolution() const;

    //! \param s: solution to be filled with a random valid solution
    virtual ga::Solution* CreateRandomSolution() const;

    void Sort(PDPSolution* solution);

    virtual size_t Size() const;

    virtual void* Data();

    //! Genetic Algorithm Crossover for pickup and delivery
    //! \param child:  solution to be filled with a random valid crossover solution.
    //! \param p1: const reference for first parent to be crossed.
    //! \param p2: const reference for seccond parent to be crossed.
    //! \return resulting crossover child.
    virtual void Crossover(ga::Solution* child, const ga::Solution* p1, const ga::Solution* p2);

    //! Repair solution
    //! \param solution: Solution to be repaired
    //! \param trace: if true, every repair step will be displayed.
    virtual void Repair(ga::Solution* solution);

    //! Mutate solution.
    virtual void Mutate(ga::Solution*);

    virtual double SolutionDistance(const ga::Solution* a, const ga::Solution* b) const;

  protected:
    //! \param s: solution to be filled with a random valid solution
    virtual int CreateRandomSolution(PDPSolution* s) const;

    //! Precompute distance between customers
    virtual void PrecomputeDistanceMatrix();

    //! Precompute n closest customers.
    //! \param size: number of closest individuals
    virtual void PrecomputeClosest(int closeindividuals);

  public:
    int* _sucessor;

    //! Maximum number of vehicles (or routes).
    const size_t vehicles;

    //! Number of nodes.
    const size_t numberOfNodes;

    //! Number of closest individuals relevant in local search
    const int nclosest;

    //! Instance comment
    const std::string comment;

    //! closest individuals maps.
    std::vector<std::vector<int> > closest;
    std::vector<double> fartherClose;

    //! list of customer nodes.
    NodeList nodes;

    virtual std::string LSCompleteLog();
    virtual std::string LSLog();
    virtual void LSLogReset();

  protected:
    //! Educate process instance
    pdp::Educate* pEducate;

  protected:
    pdp::moves::PDPMove* pRelocateMove;
    pdp::moves::PDPMove* p4optMove;

    friend class PDPInstanceReader;
    bool* _visited;
};

}  // namespace pdp

#endif  // PDPInstance_H
