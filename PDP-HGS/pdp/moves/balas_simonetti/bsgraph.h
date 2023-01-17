/*****************************************************************************
 * Copyright (C) 2016 Tulio A.M. Toffolo
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#ifdef BALAS_SIMONETTI

#ifndef BSGRAPH_H
#define BSGRAPH_H

#include <stdlib.h>
#include <time.h>

#include <list>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "bscache.h"
#include "util.h"

using namespace std;

/*===========================================================================*/
// BSNode class
/*===========================================================================*/

/**
 * This class represents a node in the layered graph used by Ballas and Simonetti
 * (2001) in their "Linear Time Dynamic-Programming Algorithms for TSPs".
 */
class BSNode {
  public:
    /**
     * Constructor for a BSNode that considers a relative position, and two sets (minus and plus),
     * already passed as bit-sets (int)
     *
     * @param k value for k in the graph
     * @param position the relative position of the node
     * @param minus bit-set (int) with the items in set S^-
     * @param plus bit-set (int) with the items in set S^+
     */
    BSNode(unsigned int k, int position, int minus, int plus);

    /// position of the node in the graph's node list
    int id;

    /// previous position (layer) in the original order
    int position;

    /// minimum size of *K* for which this node is required
    int minK;

    /// minimum (<= 0) and maximum (>= 0) variations of the relative position
    int minPosition, maxPosition;

    /// integers that store the items in sets S^- and S^+ using bits position
    int minus, plus;

    /// expanded representation of S^- and S^+ (uses more memory but permits easy iterating the
    /// item)
    vector<int> minusVec, plusVec;

    /// nodes that are predecessors of this node in the graph (set of nodes)
    std::vector<BSNode*> preds;

    /// nodes that are successors of this node in the graph (set of nodes)
    std::vector<BSNode*> succs;

    /// pre-processed successors (for a specific graph size and layer)
    std::vector<std::vector<std::vector<BSNode*>>> succsPerSizeLayer;

    /**
     * Checks (in constant time) if a position is part of the S^- set
     */
    inline bool hasMinus(int position) const;

    /**
     * Checks (in constant time) if a position is part of the S^+ set
     */
    inline bool hasPlus(int position) const;

    bool operator==(const BSNode& node) const;

    bool operator<(const BSNode& node) const;

    /**
     * Prints the node using the notation of Ballas and Simonetti (2001)
     */
    std::string toString() const;
};

/*===========================================================================*/
// BSGraph class
/*===========================================================================*/

/**
 * This class represents the layered graph used by Ballas and Simonetti (2001) in
 * their "Linear Time Dynamic-Programming Algorithms for TSPs".
 */
class BSGraph {
  public:
    /**
     * Constructor that creates an empty graph considering a certain k.
     *
     * @param params Params object reference for converting clients indices
     * @param k "move range" considered to create the graph
     * @param total number of clients in the problem
     * @param distMatrix original matrix with the distances between every two clients
     * @param maxTime the time in which the algorithm MUST stop running
     */
    BSGraph(unsigned int k, unsigned int nLocations, double** distMatrix, clock_t maxTime, size_t maxMemory);

    /// singleton object
    static BSGraph* singleton;

    /// the k factor used to generate the graph
    unsigned int k;

    /// pointer do the matrix with the costs
    double** distMatrix;

    /// maximum runtime (latest clock tick)
    clock_t maxClock;

    /// maxmimum memory the cache may use
    const size_t maxMemory;

    /// the maximum size (maximum number of "non-repeated" layers) of the graph
    /// this value is generally a constant: size = (k*2) + 1
    unsigned int maxSize;

    /// number of edges in a complete layer of the graph
    unsigned int nEdges;

    /// number of locations
    unsigned int nLocations;

    /// nodes of a complete layer of the graph
    std::vector<BSNode*> nodes;

    /// number of necessary nodes to process a sequence with *size* nodes
    std::vector<unsigned int> nNodesPerSize;

    /// pre-processed nodes of a specific graph considering *size* and *layer*
    std::vector<std::vector<std::vector<BSNode*>>> nodesPerSizeLayer;

    /// nodes per class (a class is given by a pair (S^-,S^+))
    std::vector<std::vector<BSNode*>> nodeClasses;

    /// distance of the last shortest path computed
    double distance;

    /// basic usage statistics
    size_t cacheQueries, cacheImprovement, cacheSideways, cacheRemoval, cacheCollisions, cacheCleanUps;

    /**
     * Gets the individual hashCode of the customer given as argument.
     *
     * @param n index of the customer
     */
    inline size_t getClientHash(int n) const {
      return cachedHashCodes[n];
    }

    /**
     * Gets the individual ('reverse') hashCode of the customer given as argument.
     *
     * @param n index of the customer
     */
    inline size_t getClientHashRev(int n) const {
      return cachedHashCodesRev[cachedMultipliers.size() - 1 - n];
    }

    /**
     *
     */
    inline double getDistance(int source, int destination) const {
      return distMatrix[source][destination];
    }

    /**
     * Gets the hashCode multiplier of a specific position.
     */
    inline size_t getHashMultiplier() const {
      return hashMultiplier;
    }

    /**
     * Gets the hashCode multiplier of a specific position.
     *
     * @param n index of the customer
     */
    inline size_t getHashMultiplier(int position) const {
      return cachedMultipliers[position];
    }

    /**
     * Gets the cost of and edge (v1,v2) considering a relative layer.
     *
     * @param sequence (initial) ordered vector with clients' id
     * @param layer layer under analysis (important to calculate position)
     * @param pred node of the current layer (predecessor)
     * @param succ node of the next layer (successor)
     */
    inline double getEdgeCost(const std::vector<int>& sequence, int layer, BSNode* pred, BSNode* succ) const;

    /**
     * Calculates the best ordering for a sequence considering up to k changes.
     * The calculation is equivalent to finding the shortest path in a layered
     * graph, and takes O(|E|), where |E| is the number of edges in the graph.
     * Note that the sequence given as argument is updated.
     *
     * @param sequence ordered vector with clients to update
     * @param firstIndex index of the first item that may be changed
     * @param lastIndex index of the last item that may be changed
     * @param inputDistance initial distance
     * @param hashCode hash code of the initial sequence
     *
     * @return true if route is improved and false otherwise
     */
    bool runBS(std::vector<int>& sequence, const int firstIndex, const int lastIndex,
               const double inputDistance, const BSHash& hash);

    /**
     * Wrapper functions to calculate the best ordering of a sequence considering up to k changes.
     *
     * @param sequence ordered vector with clients to update
     * @param firstIndex index of the first item that may be changed
     * @param lastIndex index of the last item that may be changed
     *
     * @return true if route is improved and false otherwise
     */
    bool updateOrder(vector<int>& sequence, double& updatedDistance);
    bool updateOrder(std::vector<int>& sequence, const int firstIndex, const int lastIndex);
    bool updateOrder(std::vector<int>& sequence, const int firstIndex, const int lastIndex,
                     bool& doubleCheck);

    /**
     * Creates a hash value (given by three numbers) for an input given sequence.
     *
     * @param sequence ordered vector with clients
     * @param firstIndex index of the first item that may be changed
     * @param lastIndex index of the last item that may be changed
     * @param distance total distance of original route
     *
     * @return the BSHash for the input given sequence.
     */
    BSHash makeBSHash(const std::vector<int>& sequence, const int firstIndex, const int lastIndex,
                      const double distance);

    ~BSGraph();

  private:
    /// main cache multiplier (used within the operations)
    size_t hashMultiplier;

    /// cached hash values for customers and multipliers
    std::vector<size_t> cachedHashCodes, cachedHashCodesRev, cachedMultipliers;

    /// hash list to maintain already processed sequences
#ifdef NO_TUNNELING
    std::unordered_map<BSHash, BSCache*, BSHash> cache;
#else
    std::unordered_map<BSSetHash, BSCache*, BSSetHash> cache;
#endif
    /// vector to maintain last used cached values
    std::vector<BSCache*> cacheVector;

    /// counter to control memory usage of the cache
    size_t cacheMemoryUsage;

    /// auxiliary structures used to compute the shortest paths
    double *prevDists, *dists;

    /// auxiliary matrix that keeps the shortest paths
    std::vector<std::vector<BSNode*>> shortestPaths;

    /**
     * Removes elements from the cache to open up space for new elements.
     */
    void clearCache();

    /**
     * Creates the arcs between nodes (it is mandatory to call createNodes()
     * before calling this method).
     */
    void createArcs();

    /**
     * Creates the nodes for a given k.
     */
    void createNodes();

    /**
     * Pre-process the graph, storing the nodes of each layer. This prevents
     * unnecessary comparisons when computing the shortest path.
     */
    void preProcessLayers();

    /**
     * Returns a cached solution (if any exists) for a given input.
     */
    bool updateFromCache(const BSHash& hash, std::vector<int>& sequence, const int begin, const int end,
                         bool& doubleCheck);

  private:
    vector<int> result_vec;
};

#endif  // BALAS_SIMONETTI
#endif  // BSGRAPH_H
