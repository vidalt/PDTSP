/*****************************************************************************
 * Copyright (C) 2016 Túlio A.M. Toffolo
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

#include "bsgraph.h"
#include "time.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <queue>
#include <sstream>

#include "pdp/pdpnode.h"
#include "pdp/pdproute.h"
#include "time.h"
#include "utils/application.h"

using namespace pdp;
using namespace std;

// temporary (debugging) stuff...
#define BSGRAPH_DEBUG 0
#define CACHE_MAX 60000011
#define CACHE_OCCUPATION 0.8
#define CACHE_RESULTS 0

// initializing singleton object to null
BSGraph *BSGraph::singleton = nullptr;

/*===========================================================================*/
// BSNode class
/*===========================================================================*/

BSNode::BSNode(unsigned int k, int position, int minus, int plus) {
  this->id = -1;  // to indicate that the id is not set
  this->position = position;
  this->minus = minus;
  this->plus = plus;

  minPosition = min(position, 0);
  maxPosition = max(position, 0);

  // updating sMinus
  for (int i = 0; i < (int)k; i++) {
    if (hasMinus(i)) {
      maxPosition = max(maxPosition, i);
      minusVec.push_back(i);
    }
  }

  // updating sPlus
  for (int i = 1; i < (int)k; i++) {
    if (hasPlus(i)) {
      minPosition = min(minPosition, -i);
      plusVec.push_back(i);
    }
  }

  minK = max(max(abs(maxPosition) + abs(minPosition) + 1, abs(position) + 1), 1);
}

bool BSNode::hasMinus(int position) const {
  return (minus & (1 << (abs(position)))) != 0;
}

bool BSNode::hasPlus(int position) const {
  return (plus & (1 << abs(position))) != 0;
}

bool BSNode::operator==(const BSNode &node) const {
  return position == node.position && minus == node.minus && plus == node.plus;
}

bool BSNode::operator<(const BSNode &node) const {
  if (id != node.id) return id < node.id;

  return minK < node.minK;
}

std::string BSNode::toString() const {
  string result;
  result.reserve(30);

  result += "i";
  if (position != 0) result.append(string_format("%c%d", position > 0 ? '+' : '-', abs(position)));

  string sMinusStr = "{";
  for (int s = 0; s < minK; s++) {
    if (hasMinus(s)) {
      if (sMinusStr != "{") sMinusStr.append(" ");

      if (s == 0)
        sMinusStr.append("i");
      else
        sMinusStr.append(string_format("i+%d", s));
    }
  }
  sMinusStr.append("}");

  string sPlusStr = "{";
  for (int s = minK - 1; s >= 0; s--) {
    if (hasPlus(s)) {
      if (sPlusStr != "{") sPlusStr.append(" ");

      if (s == 0)
        sPlusStr.append("i");
      else
        sPlusStr.append(string_format("i-%d", s));
    }
  }
  sPlusStr.append("}");

  return string_format("%-4d (%s, %s, %s)", id + 1, result.c_str(), sMinusStr.c_str(), sPlusStr.c_str());
}

/*===========================================================================*/
// BSGraph class
/*===========================================================================*/

BSGraph::BSGraph(unsigned int k, unsigned int nLocations, double **distMatrix, clock_t maxClock,
                 const size_t maxMemory)
    : maxMemory(maxMemory) {
  this->k = k;
  this->distMatrix = distMatrix;
  this->maxClock = maxClock;
  this->maxSize = (k * 2) + 1;
  this->nEdges = 0;
  this->nLocations = nLocations;

  this->result_vec.resize(nLocations + 2);

  // printf("Creating Ballas&Simonetti heuristic with k=%u for %d locations.\n",
  // k, nLocations); printf("    Cache maximum memory: %.0fMb\n\n", (double)
  // maxMemory / 1e6);

  // creating nodes and connecting them
  createNodes();
  createArcs();

  // pre-processing the nodes in each layer -- to avoid useless comparison
  preProcessLayers();

  // initializing auxiliary structures
  prevDists = new double[nodes.size()];
  dists = new double[nodes.size()];

  // initializing (and caching) multipliers (for faster hash computation)
  hashMultiplier =
      next_prime(nLocations + 1);  // +1 because depots may be duplicated (first and last stop-point)
  cachedMultipliers.assign(nLocations + 1, 1);
  for (unsigned int i = 1; i < nLocations + 1; i++)
    cachedMultipliers[i] = (cachedMultipliers[i - 1] * hashMultiplier) % BIG_PRIME;

  // calculating and caching individual hash codes for each depot/client
  cachedHashCodes.assign(nLocations + 1, 1);
  for (unsigned int i = 1; i < nLocations + 1; i++)
    cachedHashCodes[i] = (cachedMultipliers[i] * (i + 1));  // % BIG_PRIME;

  // calculating and caching individual hash codes for each depot/client (second
  // hash function)
  cachedHashCodesRev.assign(nLocations + 1, 1);
  for (unsigned int i = 1; i < nLocations + 1; i++)
    cachedHashCodesRev[i] = (cachedHashCodesRev[i - 1] * 31 + (i + 1));  // % BIG_PRIME;

  if (CACHE_RESULTS) {
    // allocating space and resetting cache counters
    cacheMemoryUsage = 0;
    cache.reserve(CACHE_MAX);
    cacheVector.reserve((size_t)ceil(CACHE_MAX * CACHE_OCCUPATION) + 1);
    cacheQueries = 0;
    cacheCleanUps = 0;
    cacheImprovement = 0;
    cacheSideways = 0;
    cacheRemoval = 0;
    cacheCollisions = 0;

    cacheMemoryUsage = sizeof(void *) * cache.bucket_count()       // cache size
                       + sizeof(void *) * cacheVector.capacity();  // cacheVector size
  }

  if (BSGRAPH_DEBUG) {
    // printing number of nodes (simple check)
    printf("Number of nodes per layer: %zu\n", nodes.size());
    printf("Number of edges per layer: %u\n", nEdges);

    // printing nodes and their connections
    for (vector<BSNode *>::iterator nodeIt = nodes.begin(); nodeIt != nodes.end(); nodeIt++) {
      BSNode *node = *nodeIt;
      if (node->id >= 99) {
        printf(".....\n\n");
        break;
      }
      printf("%s\n", node->toString().c_str());
      for (vector<BSNode *>::iterator succIt = node->succs.begin(); succIt != node->succs.end(); succIt++) {
        BSNode *succ = *succIt;
        printf("  --> %s\n", succ->toString().c_str());
      }
    }
  }

  singleton = this;
}

void BSGraph::clearCache() {
  if (cacheVector.size() < CACHE_MAX * CACHE_OCCUPATION && cacheMemoryUsage < maxMemory) return;

  clock_t start = clock();
  printf("cleaning cache (~%.2f gb used)...\n", (double)cacheMemoryUsage / 1e9);
  assert(CACHE_RESULTS);
  cacheCleanUps++;

  size_t newSize = cacheVector.size() / 2;
  nth_element(cacheVector.begin(), cacheVector.begin() + newSize, cacheVector.end(), BSCache::higherUsage);

  // important: all elements after newSize have usage <= the median and should
  // be removed
  while (cacheVector.size() > newSize) {
    BSCache *element = cacheVector.back();
    cacheMemoryUsage -= element->memorySize();
    cacheVector.pop_back();
    cache.erase(element->getHash());
    delete element;
  }

  // updating indices of cached objects
  for (size_t i = 0; i < cacheVector.size(); i++) {
    cacheVector[i]->index = i;
    cacheVector[i]->usage = cacheVector[i]->usage == 0 ? 0 : 1;
  }
  printf("done in %.2fs!\n\n", (double)(clock() - start) / CLOCKS_PER_SEC);
}

void BSGraph::createArcs() {
  // as stated in Balas and Simonetti (2001), the nodes can be grouped
  // by (S-,S+), reducing the number of comparisons for creating arcs
  for (vector<BSNode *>::iterator predIt = nodes.begin(); predIt != nodes.end(); predIt++) {
    BSNode *pred = *predIt;
    for (vector<vector<BSNode *> >::iterator nodeClassIt = nodeClasses.begin();
         nodeClassIt != nodeClasses.end(); nodeClassIt++) {
      vector<BSNode *> &nodeClass = *nodeClassIt;

      BSNode *succ = nodeClass[0];
      bool adj = false;

      if (pred->position < 0) {  // node is of type (i+X, ...)
        if (pred->minus & 1) {
          adj = ((succ->minus << 1) == (pred->minus - 1)) &&
                ((succ->plus >> 1) == (pred->plus - (1 << (-pred->position))));
        } else {
          adj = ((succ->minus << 1) == pred->minus) &&
                ((succ->plus >> 1) == (pred->plus + 1 - (1 << (-pred->position))));
        }
      } else if (pred->position > 0) {  // node is of type (i-X, ...)
        if (pred->minus & 1) {
          adj = ((succ->plus >> 1) == pred->plus) &&
                ((succ->minus << 1) == (pred->minus - 1 + (1 << (pred->position))));
        } else {
          adj = ((succ->plus >> 1) == (pred->plus + 1)) &&
                ((succ->minus << 1) == (pred->minus + (1 << (pred->position))));
        }
      } else {  // node is of type (i, ...)
        adj = ((succ->plus >> 1) == pred->plus) && ((succ->minus << 1) == pred->minus);
      }

      if (adj) {
        nEdges += nodeClass.size();
        pred->succs = nodeClass;
        for (vector<BSNode *>::iterator succIt = nodeClass.begin(); succIt != nodeClass.end(); succIt++) {
          BSNode *successor = *succIt;
          successor->preds.push_back(pred);
        }
      }
    }
  }
}

void BSGraph::createNodes() {
  // calculating total number of combinations for set *plus* (2*2^k)
  int nPlusCombinations = (2 << (k - 1));
  for (int plus = 0; plus < nPlusCombinations; plus += 2) {
    // computing size and last element in set *plus*
    int plusSize = count_bits(plus);
    int lastPlus = plus == 0 ? 0 : fls(plus) - 1;

    // if current set is too large, then move to the next one
    if (plusSize > (int)k / 2) continue;

    // calculating total number of combinations for set *minus* (2^(k-lastPlus))
    int nMinusCombinations = 1 << (k - lastPlus);
    for (int minus = 0; minus < nMinusCombinations; minus++) {
      // computing size of set *minus* (must be equal to size of set *plus)
      if (plusSize != count_bits(minus)) continue;

      // indicates if any node is generated with the considered S^- and S^+
      bool newClass = true;

      // creating a node per item in *plus* (i-X)
      for (int j = 1; j < (int)k; j++) {
        if (plus & (1 << j)) {
          BSNode *node = new BSNode(k, -j, minus, plus);
          nodes.push_back(node);

          if (newClass) {
            newClass = false;
            nodeClasses.push_back(vector<BSNode *>());
          }
          nodeClasses.back().push_back(node);
        }
      }

      // creating positive nodes (i to i+(k-lastPlus-1))
      for (int j = 0; j < (int)k - lastPlus; j++) {
        if (!(minus & (1 << j))) {
          BSNode *node = new BSNode(k, j, minus, plus);
          nodes.push_back(node);

          if (newClass) {
            newClass = false;
            nodeClasses.push_back(vector<BSNode *>());
          }
          nodeClasses.back().push_back(node);
        }
      }
    }
  }

  // sorting nodes according to their minK and setting their ids
  stable_sort(nodes.begin(), nodes.end(), lessptr<BSNode *>());
  for (unsigned int i = 0; i < nodes.size(); i++)
    nodes[i]->id = i;
  for (vector<BSNode *>::iterator nodeIt = nodes.begin(); nodeIt != nodes.end(); nodeIt++) {
    BSNode *node = *nodeIt;
    sort(node->preds.begin(), node->preds.end(), lessptr<BSNode *>());
    sort(node->succs.begin(), node->succs.end(), lessptr<BSNode *>());
  }
}

double BSGraph::getEdgeCost(const vector<int> &sequence, int layer, BSNode *pred, BSNode *succ) const {
  if (layer == 0 || layer == (int)sequence.size()) return 0;

  PDPNode **nodes = static_cast<PDPNode **>(Application::instance->Data());

  int prevLayer = layer - 1 + pred->position;
  int nextLayer = layer + succ->position;

  // checking invalid exchanges using deliveries in minusVec
  for (int minus : succ->minusVec) {
    PDPNode *minusNode = nodes[sequence[layer + minus]];
    if (minusNode->isDelivery) {
      // checking exchange in *minus* with current node
      if (minusNode->pair == sequence[nextLayer]) return (double)INFINITY;

      // checking exchange in *minus* with future allocations (*plus*)
      for (int plus : succ->plusVec)
        if (minusNode->pair == sequence[layer - plus]) return (double)INFINITY;
    }
  }

  // checking invalid exchanges using pickups in plusVec (only against current
  // node)
  for (int plus : succ->plusVec) {
    PDPNode *plusNode = nodes[sequence[layer - plus]];
    if (plusNode->isPickup && plusNode->pair == sequence[nextLayer]) return (double)INFINITY;
  }

  return getDistance(sequence[prevLayer], sequence[nextLayer]);
}

BSHash BSGraph::makeBSHash(const vector<int> &sequence, const int firstIndex, const int lastIndex,
                           const double distance) {
  // calculating hash of sequence
  int begin = (firstIndex - 1 >= 0 && sequence[firstIndex - 1] == 0) ? firstIndex - 1 : firstIndex;
  int end =
      (lastIndex + 1 < (int)sequence.size() && sequence[lastIndex + 1] == 0) ? lastIndex + 1 : lastIndex;

  // calculating hash values
  size_t seqHash = (size_t)sequence[begin] + 1;
  size_t seqHashRev = (size_t)sequence[end] + 1;
  size_t setHash = getClientHash(sequence[begin]);
  size_t setHashRev = getClientHashRev(sequence[begin]);

  for (int i = begin + 1; i <= end; i++) {
    seqHash = (seqHash + cachedMultipliers[i - begin] * (sequence[i] + 1)) % BIG_PRIME;
    seqHashRev = (seqHashRev + cachedMultipliers[i - begin] * (sequence[end - (i - begin)] + 1)) % BIG_PRIME;
    setHash = (setHash + getClientHash(sequence[i]));           // % BIG_PRIME;
    setHashRev = (setHashRev + getClientHashRev(sequence[i]));  // % BIG_PRIME;
  }

  return BSHash(seqHash, seqHashRev, setHash, setHashRev, distance, end - begin + 1);
}

void BSGraph::preProcessLayers() {
  // auxiliary variable to store the last necessary node
  int lastUsedNode = 0;

  // assigning value to maxSize and pre-allocating vectors
  nNodesPerSize.assign(maxSize + 1, 0);
  nodesPerSizeLayer.assign(maxSize + 1, vector<vector<BSNode *> >());
  for (vector<BSNode *>::iterator nodeIt = nodes.begin(); nodeIt != nodes.end(); nodeIt++) {
    BSNode *node = *nodeIt;
    node->succsPerSizeLayer.assign(maxSize + 1, vector<vector<BSNode *> >());
  }

  // pre-computing the nodes present in each layer and their successors
  for (int size = 0; size <= (int)maxSize; size++) {
    // pre-allocating vectors
    nodesPerSizeLayer[size].assign((unsigned int)size, vector<BSNode *>());
    for (vector<BSNode *>::iterator nodeIt = nodes.begin(); nodeIt != nodes.end(); nodeIt++) {
      BSNode *node = *nodeIt;
      node->succsPerSizeLayer[size].assign((unsigned int)size, vector<BSNode *>());
    }

    for (int layer = 0; layer < size; layer++) {
      for (vector<BSNode *>::iterator nodeIt = nodes.begin(); nodeIt != nodes.end(); nodeIt++) {
        BSNode *node = *nodeIt;
        if (node != nodes[0] && (node->minPosition + layer <= 0 || node->maxPosition + layer >= size))
          continue;

        lastUsedNode = max(lastUsedNode, node->id);

        nodesPerSizeLayer[size][layer].push_back(node);
        for (vector<BSNode *>::iterator succIt = node->succs.begin(); succIt != node->succs.end(); succIt++) {
          BSNode *succ = *succIt;
          if (succ != nodes[0] &&
              (succ->minPosition + layer + 1 <= 0 || succ->maxPosition + layer + 1 >= size))
            continue;

          node->succsPerSizeLayer[size][layer].push_back(succ);
        }
      }
    }

    // calculating number of necessary nodes for a sequence *size*
    nNodesPerSize[size] = (unsigned int)lastUsedNode + 1;
  }

  if (BSGRAPH_DEBUG) {
    // printing nodes in each layer
    int maxPrintSize = maxSize <= 7 ? maxSize : 7;
    for (int size = 0; size <= maxPrintSize; size++) {
      printf("size: %d\n", size);
      for (int layer = 0; layer < size; layer++) {
        printf("layer[%d] = ", layer);
        for (vector<BSNode *>::iterator nodeIt = nodesPerSizeLayer[size][layer].begin();
             nodeIt != nodesPerSizeLayer[size][layer].end(); nodeIt++) {
          BSNode *node = *nodeIt;
          printf("%d", node->id + 1);
          if (!node->succsPerSizeLayer[size][layer].empty()) {
            printf("[");
            for (vector<BSNode *>::iterator succIt = node->succsPerSizeLayer[size][layer].begin();
                 succIt != node->succsPerSizeLayer[size][layer].end(); succIt++) {
              BSNode *succ = *succIt;
              printf("%d,", succ->id + 1);
            }
            printf("]");
          }
          printf(" ");
        }
        printf("\n");
      }
      printf("\n");
    }
  }
}

bool BSGraph::runBS(vector<int> &sequence, const int firstIndex, const int lastIndex,
                    const double inputDistance, const BSHash &hash) {
  if (lastIndex - firstIndex <= 2) return false;

  distance = inputDistance;
  if (firstIndex >= lastIndex) return false;
  // if (clock() > maxClock) return false;

  if (BSGRAPH_DEBUG) {
    // calculating and printing the initial cost
    double initialCost = 0;
    for (unsigned int i = 1; i < sequence.size(); i++) {
      int prevLayer = i - 1;
      int nextLayer = i == sequence.size() ? 0 : i;
      initialCost += getDistance(sequence[prevLayer], sequence[nextLayer]);
    }
    printf("initial cost = %.2f\n", initialCost);
  }

  int size = lastIndex - firstIndex + 2;
  int sizeIdx = min(size, (int)maxSize);

  // creating (if necessary) *shortestPaths* structure using O(size *
  // nodes.size()) memory
  if (shortestPaths.size() <= (unsigned int)size)
    shortestPaths.assign((unsigned int)size + 1, vector<BSNode *>(nodes.size()));

  // computing cost of the fixed part of the sequence
  double fixedCost = 0;
  for (int i = 1; i < firstIndex; i++)
    fixedCost += getDistance(sequence[i - 1], sequence[i]);
  for (int i = lastIndex + 1; i < (int)sequence.size() - 1; i++)
    fixedCost += getDistance(sequence[i], sequence[i + 1]);

  bool improvement = true;
  while (improvement) {
    // if (clock() > maxClock) break;

    // setting initial values of auxiliary structure *_costs*
    fill_n(dists, nNodesPerSize[sizeIdx], fixedCost);

    // computing (forward) shortest paths for the layered graph in O(edges)
    int layerIdx;
    for (int layer = firstIndex; layer <= lastIndex + 1; layer++) {
      swap(prevDists, dists);
      fill_n(dists, nNodesPerSize[sizeIdx], INFINITY);

      // defining layer index (for nodes and successors fast retrieval)
      if (size <= (int)maxSize) {
        layerIdx = layer - firstIndex;
      } else {
        layerIdx = k;
        if (layer - firstIndex < (int)k)
          layerIdx = layer - firstIndex;
        else if (lastIndex + 1 - layer < (int)k)
          layerIdx = 2 * k - (lastIndex + 1 - layer);
      }

      for (vector<BSNode *>::iterator nodeIt = nodesPerSizeLayer[sizeIdx][layerIdx].begin();
           nodeIt != nodesPerSizeLayer[sizeIdx][layerIdx].end(); nodeIt++) {
        BSNode *node = *nodeIt;
        for (vector<BSNode *>::iterator succIt = node->succsPerSizeLayer[sizeIdx][layerIdx].begin();
             succIt != node->succsPerSizeLayer[sizeIdx][layerIdx].end(); succIt++) {
          BSNode *succ = *succIt;
          if (dists[succ->id] > getEdgeCost(sequence, layer, node, succ) + prevDists[node->id]) {
            dists[succ->id] = getEdgeCost(sequence, layer, node, succ) + prevDists[node->id];
            shortestPaths[layer - firstIndex][succ->id] = node;
          }
        }
      }
    }

    // updating distance (for future use)
    improvement = dists[0] < distance - EPS;
    distance = dists[0];

    // computing resulting sequence (only for the 'optimized' subsequence)
    // result_vec.resize((unsigned int) size);
    vector<int> &result = result_vec;
    BSNode *node = shortestPaths[lastIndex - firstIndex + 1][0];
    for (int i = lastIndex; i >= firstIndex; i--) {
      result[i - firstIndex] = sequence[i + node->position];
      node = shortestPaths[i - firstIndex][node->id];
    }

    // updating original sequence
    for (int i = lastIndex; i >= firstIndex; i--)
      sequence[i] = result[i - firstIndex];
  }

  // checking if there was any improvement during the search
  improvement = distance < inputDistance - EPS;

  // caching result
  if (CACHE_RESULTS) {
#ifdef NO_TUNNELING
    BSCache *cachedResult = new BSCache(hash);
    int begin = (firstIndex - 1 >= 0 && sequence[firstIndex - 1] == 0) ? firstIndex - 1 : firstIndex;
    int end =
        (lastIndex + 1 < (int)sequence.size() && sequence[lastIndex + 1] == 0) ? lastIndex + 1 : lastIndex;

    cachedResult->distance = distance;
    if (improvement) {
      cachedResult->sequence = new int[end - begin + 1];
      for (int i = begin; i <= end; i++)
        cachedResult->sequence[i - begin] = params->correspondanceTable[sequence[i]];
    }

    clearCache();

    cache[hash] = cachedResult;
    cachedResult->index = cacheVector.size();
    cacheVector.push_back(cachedResult);
    cacheMemoryUsage += cachedResult->memorySize();
#else
    BSCache *cachedResult = nullptr;
    int begin = (firstIndex - 1 >= 0 && sequence[firstIndex - 1] == 0) ? firstIndex - 1 : firstIndex;
    int end =
        (lastIndex + 1 < (int)sequence.size() && sequence[lastIndex + 1] == 0) ? lastIndex + 1 : lastIndex;

    // querying for a previous cached sequence for the current set of customers
    auto mapIt = cache.find(hash.setHash);

    if (mapIt == cache.end()) {
      // creating a new cache entry since none exists for this set of customers
      cachedResult = new BSCache(hash.setHash);

      cachedResult->distance = distance;
      cachedResult->sequence = new int[end - begin + 1];
      for (int i = begin; i <= end; i++)
        cachedResult->sequence[i - begin] = sequence[i];

      clearCache();

      cache[cachedResult->getHash()] = cachedResult;
      cachedResult->index = cacheVector.size();
      cacheVector.push_back(cachedResult);
    }

    else {
      cachedResult = mapIt->second;
      cacheMemoryUsage -= cachedResult->memorySize();

      if (distance <= mapIt->second->distance - EPS) {
        // updating old cache entry with the improved sequence

        cachedResult->distance = distance;
        for (int i = begin; i <= end; i++)
          cachedResult->sequence[i - begin] = sequence[i];
      }

      else if (mapIt->second->distance <= distance - EPS) {
        // using the sequence stored in the cache since it is better than the
        // sequence produced
        cachedResult = mapIt->second;

        // updating sequence and distance
        for (int i = begin; i <= end; i++)
          sequence[i] = cachedResult->sequence[i - begin];
        distance = cachedResult->distance;
        improvement = true;
      }
    }

    cachedResult->insert(hash.routeHash);
    cacheMemoryUsage += cachedResult->memorySize();
#endif
  }

  return improvement;
}

bool BSGraph::updateFromCache(const BSHash &hash, vector<int> &sequence, const int begin, const int end,
                              bool &doubleCheck) {
  cacheQueries++;
#ifdef NO_TUNNELING
  std::unordered_map<BSHash, BSCache *, BSHash>::const_iterator mapIt = cache.find(hash);
#else
  std::unordered_map<BSSetHash, BSCache *, BSSetHash>::const_iterator mapIt = cache.find(hash.setHash);
#endif
  if (mapIt == cache.end()) return false;

  BSCache *element = mapIt->second;
#ifdef NO_TUNNELING
#else
  if (!element->has(hash.routeHash)) return false;
#endif

  if (element->distance <= distance + EPS) {
#ifdef NO_TUNNELING
    try {
      if (doubleCheck && element->sequence != nullptr) {
        // double-checking element-set
        stringstream error;
        if (end - begin + 1 > 0 && mapIt->first.setHash.size != end - begin + 1) {
          error << "mapIt->first.setHash.size != end - begin + 1: " << mapIt->first.setHash.size << " vs "
                << (end - begin + 1);
          doubleCheck = false;
          // throw std::runtime_error(error.str());
        }
        set<int> items;
        for (int i = begin; i <= end; i++)
          items.insert(params->correspondanceTable2[element->sequence[i - begin]]);
        for (int i = begin; i <= end; i++) {
          if (items.count(sequence[i]) != 1) {
            set<int> second;
            for (int j = begin; j <= end; j++)
              second.insert(sequence[j]);

            error << "hash failure! sets are different (for " << params->nbClients << " clients):";
            for (int idx : items)
              error << " " << idx;
            error << " versus";
            for (int idx : second)
              error << " " << idx;
            // throw std::runtime_error(error.str());
            doubleCheck = false;
          }
        }

        if (!doubleCheck) {
          cerr << endl << error.str() << endl;
          cout << endl << error.str() << endl;

          assert(element == cacheVector[element->index]);

          // moving last element to position element->index
          cacheVector[element->index] = cacheVector.back();
          cacheVector[element->index]->index = element->index;
          cacheVector.pop_back();

          // freeing memory
          cache.erase(element->getHash());
          delete element;
          return false;
        }
      }
    } catch (int errorCode) {
      cerr << endl << "Fatal error with code " << errorCode << endl;
      cout << endl << "Fatal error with code " << errorCode << endl;

      // moving last element to position element->index
      cacheVector[element->index] = cacheVector.back();
      cacheVector[element->index]->index = element->index;
      cacheVector.pop_back();

      // freeing memory
      cache.erase(element->getHash());
      delete element;
      return false;
    }
#else
    try {
      if (doubleCheck) {
        // double-checking element-set
        stringstream error;
        if (end - begin + 1 > 0 && mapIt->first.size != end - begin + 1) {
          error << "mapIt->first.size != end - begin + 1: " << mapIt->first.size << " vs "
                << (end - begin + 1);
          doubleCheck = false;
          // throw std::runtime_error(error.str());
        }
        set<int> items;
        for (int i = begin; i <= end; i++)
          items.insert(element->sequence[i - begin]);
        for (int i = begin; i <= end; i++) {
          if (items.count(sequence[i]) != 1) {
            set<int> second;
            for (int j = begin; j <= end; j++)
              second.insert(sequence[j]);

            error << "hash failure! sets are different (for " << nLocations << " clients):";
            for (int idx : items)
              error << " " << idx;
            error << " versus";
            for (int idx : second)
              error << " " << idx;
            // throw std::runtime_error(error.str());
            doubleCheck = false;
          }
        }

        if (!doubleCheck) {
          cerr << endl << error.str() << endl;
          cout << endl << error.str() << endl;

          assert(element == cacheVector[element->index]);

          // moving last element to position element->index
          cacheVector[element->index] = cacheVector.back();
          cacheVector[element->index]->index = element->index;
          cacheVector.pop_back();

          // freeing memory
          cacheMemoryUsage -= element->memorySize();
          cache.erase(element->getHash());
          delete element;
          return false;
        }
      }
    } catch (int errorCode) {
      cerr << endl << "Fatal error with code " << errorCode << endl;
      cout << endl << "Fatal error with code " << errorCode << endl;

      // moving last element to position element->index
      cacheVector[element->index] = cacheVector.back();
      cacheVector[element->index]->index = element->index;
      cacheVector.pop_back();

      // freeing memory
      cacheMemoryUsage -= element->memorySize();
      cache.erase(element->getHash());
      delete element;
      return false;
    }
#endif

    if (element->distance >= distance - EPS || element->sequence == nullptr) {
      cacheSideways++;
    } else {
      // updating sequence and distance
      for (int i = begin; i <= end; i++)
        sequence[i] = element->sequence[i - begin];
      distance = element->distance;
      cacheImprovement++;
    }

    element->usage++;
    return true;
  }

  else {
#ifdef NO_TUNNELING
    assert(element == cacheVector[element->index]);

    // moving last element to position element->index
    cacheVector[element->index] = cacheVector.back();
    cacheVector[element->index]->index = element->index;
    cacheVector.pop_back();

    // freeing memory
    cache.erase(element->getHash());
    delete element;

    cacheRemoval++;
    return false;
#else
    return false;
#endif
  }
}

bool BSGraph::updateOrder(vector<int> &sequence, double &updatedDistance) {
  bool result = updateOrder(sequence, 1, (int)sequence.size() - 2);
  if (result) updatedDistance = distance;
  return result;
}

bool BSGraph::updateOrder(vector<int> &sequence, const int firstIndex, const int lastIndex) {
  bool doubleCheck = rand() % 1000 <= 1;
  return this->updateOrder(sequence, firstIndex, lastIndex, doubleCheck);
}

bool BSGraph::updateOrder(vector<int> &sequence, const int firstIndex, const int lastIndex,
                          bool &doubleCheck) {
  // calculating distance (necessary even if indices are invalid)
  distance = 0;
  for (int i = 1; i < (int)sequence.size(); i++)
    distance += getDistance(sequence[i - 1], sequence[i]);

  if (firstIndex >= lastIndex) return false;

  // checking for previous calculated sequence (and maybe replacing by current)
  BSHash hash = makeBSHash(sequence, firstIndex, lastIndex, distance);
  if (CACHE_RESULTS) {
    double prevDistance = distance;
    int begin = (firstIndex - 1 >= 0 && sequence[firstIndex - 1] == 0) ? firstIndex - 1 : firstIndex;
    int end =
        (lastIndex + 1 < (int)sequence.size() && sequence[lastIndex + 1] == 0) ? lastIndex + 1 : lastIndex;
    if (updateFromCache(hash, sequence, begin, end, doubleCheck)) return distance <= prevDistance - EPS;
  }

  return runBS(sequence, firstIndex, lastIndex, distance, hash);
}

BSGraph::~BSGraph() {
  for (vector<BSCache *>::iterator cacheIt = cacheVector.begin(); cacheIt != cacheVector.end(); cacheIt++)
    delete *cacheIt;
  cacheVector.clear();

  for (vector<BSNode *>::iterator nodeIt = nodes.begin(); nodeIt != nodes.end(); nodeIt++)
    delete *nodeIt;
  nodes.clear();

  delete[] prevDists;
  delete[] dists;
}

#endif  // BALAS_SIMONETTI
