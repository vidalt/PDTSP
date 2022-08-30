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

#ifndef PDP4optMove_H
#define PDP4optMove_H

#include "pdp/moves/pdpmove.h"

typedef struct _PickupDeliveryInfo {
    int early;
    int later;
    _PickupDeliveryInfo() : early(-1), later(-1) {
    }
} PickupDeliveryInfo;

typedef struct _SubrouteInfo {
    PickupDeliveryInfo before;
    // PickupDeliveryInfo after;
    bool reversible = true;
    inline const _SubrouteInfo& operator=(const _SubrouteInfo& o) {
      std::memcpy(this, &o, sizeof(*this));
      return *this;
    }
} SubrouteInfo;

struct AlternatingCycle {
    int i;
    int j;
    //    int limit;
};

typedef struct _double_pair {
    double d;
    double c;
} double_pair;

typedef struct _int_pair {
    int d;
    int c;
} int_pair;

typedef struct _ac_pair {
    AlternatingCycle d;
    AlternatingCycle c;
} ac_pair;

typedef enum { MOVE_DD, MOVE_DC, MOVE_CD, MOVE_CC } MoveType;

typedef struct _segments {
    int points[10];
    int n;
    MoveType type;

    void set(int n, const int* a, const int* b, const int* c, const int* d, const int* e, MoveType type) {
      this->n = n;
      this->type = type;
      const int* blks[] = {a, b, c, d, e};
      for (int i = 0; i < n; i++) {
        points[i * 2] = blks[i][0];
        points[i * 2 + 1] = blks[i][1];
      }
    }
} Segments;

typedef struct _PDP4optMoveState {
    Segments segments;
    AlternatingCycle pred_t[2];
    double best_t;
    bool type[2];
} PDP4optMoveState;

namespace pdp {
namespace moves {

class PDP4optMove : public PDPMove {
  public:
    PDP4optMove(bool allowInfeasible = false);
    virtual ~PDP4optMove();

    bool improve(pdp::PDPRoute* route);

    //! Get local search move name.
    virtual std::string name() const;

    //! O(n) Evaluation of best move for a given route.
    //! \param solution: current Solution representation.
    //! \param routeIndex: index of route to evaluate
    //! \return PDPMoveEvaluation: evaluation containing parameters for best move found in route.
    PDPMoveEvaluation Evaluate(PDPSolution* solution, bool unfeasible);

    //! O(n) Evaluation of best move for a given route.
    //! \param solution: current Solution representation.
    //! \param routeIndex: index of route to evaluate
    //! \return PDPMoveEvaluation: evaluation containing parameters for best move found in route.
    virtual PDPMoveEvaluation Evaluate(PDPSolution* solution);

    //! O(n) Evaluation of best move for a node to a given route.
    //! \param solution: current Solution representation.
    //! \param pickupNode: pointer to Node that represents the pickup node to move.
    //! \param routeIndex: index of route to evaluate
    //! \return PDPMoveEvaluation: evaluation containing parameters for best move found in route.
    virtual PDPMoveEvaluation Evaluate(PDPSolution* solution, PDPNode* pickupNode);

    //! Apply move with given parameters.
    //! \param solution: current Solution representation to be modified.
    //! \param eval: Previous evaluation with parameters for apply move.
    //! \return double: New Solution cost after local search move.
    virtual double move(PDPSolution* solution, const PDPMoveEvaluation& eval);

    virtual size_t ResetCount();
    virtual const char* ExtraInfo() const;
    virtual const char* ExtraTotalInfo() const;

  private:
    void doMove(int* sol, int n, Segments segments);
    inline double connectSegmentsDelta(const int* sol, const int* a, const int* b, const int* c, const int* d,
                                       const int* e);

    double best4opt(int* sol, int n, int i1, int j1, int i2, int j2, double bestKnown);
    double bestFromDD(const int* sol, int blks[][2], double removedEdgesDelta, double bestKnown);
    double bestFromDC(const int* sol, int blks[][2], double removedEdgesDelta, double bestKnown);
    double bestFromCD(const int* sol, int blks[][2], double removedEdgesDelta, double bestKnown);

    void Precompute(pdp::PDPRoute* route);

    int* sol;
    int* oldSol;
    double** cost;

    double_pair* best_reach;
    double_pair* best_cross;
    int_pair* pred_reach;
    ac_pair* pred_cross;

    size_t countDD;
    size_t countDC;
    size_t countCD;
    size_t countTotalDD;
    size_t countTotalDC;
    size_t countTotalCD;

    PDP4optMoveState searchState;
    std::vector<int> customerPosition;
    std::vector<std::vector<SubrouteInfo>> subrouteInfo;
    bool allowInfeasible;
};

}  // namespace moves
}  // namespace pdp

#endif  // PDP4optMove_H
