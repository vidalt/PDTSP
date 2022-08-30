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

#ifndef BSCACHE_H
#define BSCACHE_H

#include <stdlib.h>

#include <list>
#include <set>
#include <unordered_set>
#include <vector>

#include "util.h"

/*===========================================================================*/
// BSRouteHash class
/*===========================================================================*/

struct BSRouteHash {
  public:
    /***
     * Empty constructor for a BSHash
     */
    BSRouteHash() : routeHash(0), distance(0) {
    }

    /***
     * Constructor for a BSHash
     *
     * @param routeHash hash code of the set (ignoring permutation)
     * @param routeHashRev 'inverted' hash code of the set (ignoring permutation)
     * @size number of elements in the set
     */
    BSRouteHash(const size_t& routeHash, const size_t& routeHashRev, const double& distance)
        : routeHash((routeHash << 32) + routeHashRev), distance(distance) {
    }

    inline bool operator==(const BSRouteHash& hash) const {
      return routeHash == hash.routeHash && distance == hash.distance;
    }

    /// simple BSHash -> size_t transformation (for the unordered_map)
    inline size_t operator()(const BSRouteHash& h) const {
      return h.routeHash;
    }

    /// hash code of the sequence
    const size_t routeHash;
    const double distance;
};

/*===========================================================================*/
// BSSetHash class
/*===========================================================================*/

struct BSSetHash {
  public:
    /***
     * Empty constructor for a BSHash
     */
    BSSetHash() : setHash(0), setHashRev(0), size(0) {
    }

    /***
     * Constructor for a BSHash
     *
     * @param setHash hash code of the set (ignoring permutation)
     * @param setHashRev 'inverted' hash code of the set (ignoring permutation)
     * @size number of elements in the set
     */
    BSSetHash(const size_t& setHash, const size_t& setHashRev, const int& size)
        : setHash(setHash), setHashRev(setHashRev), size(size) {
    }

    inline bool operator==(const BSSetHash& hash) const {
      return setHash == hash.setHash && setHashRev == hash.setHashRev && size == hash.size;
    }

    /// simple BSHash -> size_t transformation (for the unordered_map)
    inline size_t operator()(const BSSetHash& h) const {
      return h.setHash;
    }

    /// hash code of the sequence
    const size_t setHash;
    const size_t setHashRev;
    const int size;
};

/*===========================================================================*/
// BSHash class
/*===========================================================================*/

struct BSHash {
  public:
    /***
     * Empty constructor for a BSHash
     */
    BSHash() : routeHash(), setHash() {
    }

    /***
     * Constructor for a BSHash
     *
     * @param routeHash hash code of the route
     * @param routeHashRev 'inverted' hash code of the route
     * @param setHash hash code of the set (ignoring permutation)
     * @param setHash 'inverted' hash code of the set (ignoring permutation)
     * @param distance total distance of the route
     */
    BSHash(const size_t& routeHash, const size_t& routeHashRev, const size_t& setHash,
           const size_t& setHashRev, const double& distance, const int& size)
        : routeHash(routeHash, routeHashRev, distance), setHash(setHash, setHashRev, size) {
    }

    inline bool operator==(const BSHash& hash) const {
      return routeHash == hash.routeHash && setHash == hash.setHash;
    }

    /// simple BSHash -> size_t transformation (for the unordered_map)
    inline size_t operator()(const BSHash& h) const {
      return h.routeHash.routeHash + h.setHash.setHash;
    }

    /// hash code of the sequence
    const BSRouteHash routeHash;
    const BSSetHash setHash;
};

/*===========================================================================*/
// BSCache class
/*===========================================================================*/

#ifdef NO_TUNNELING

class BSCache {
  public:
    /***
     * Constructor for a BSCache
     *
     * @param hashCode hash code of the input sequence
     */
    BSCache(const BSHash hash) : hash(hash), usage(0) {
      sequence = nullptr;
    }

    ~BSCache() {
      if (sequence != nullptr) delete[] sequence;
    }

    inline bool operator<(const BSCache& cache) const {
      return usage < cache.usage;
    }

    inline BSHash getHash() const {
      return hash;
    }

    inline size_t memorySize() const {
      return sizeof(BSCache)                                         // this structure' memory plus
             + sizeof(BSCache*) + sizeof(BSSetHash) + sizeof(void*)  // memory used within the main cache
             + sizeof(int) * hash.setHash.size;                      // sequence memory
    }

    /// hash code of the set
    const BSHash hash;

    /// optimized (cached) sequence
    int* sequence;

    /// optimized distance
    double distance;

    /// position in the vector for O(1) removal
    size_t index;

    /// number of times this cache element was used
    size_t usage;

    /**
     * Returns true if b1 has higher usage than b2 and false otherwise
     */
    static bool higherUsage(const BSCache* b1, const BSCache* b2);

    /**
     * Runs tests with the BSCache (to check collisions, for example)
     */
    static int runTest(int argc, char* argv[]);
};

#else

class BSCache {
  public:
    /***
     * Constructor for a BSCache
     *
     * @param hashCode hash code of the input sequence
     */
    BSCache(const BSSetHash setHash) : setHash(setHash), usage(0) {
      sequence = nullptr;
    }

    ~BSCache() {
      if (sequence != nullptr) delete[] sequence;
    }

    inline bool operator<(const BSCache& cache) const {
      return usage < cache.usage;
    }

    inline BSSetHash getHash() const {
      return setHash;
    }

    inline bool has(const BSRouteHash& routeHash) const {
      return routes.count(routeHash) > 0;
    }

    inline void insert(const BSRouteHash& routeHash) {
      routes.insert(routeHash);
      usage++;
    }

    inline size_t memorySize() const {
      return sizeof(BSCache)                                         // this structure' memory plus
             + sizeof(BSCache*) + sizeof(BSSetHash) + sizeof(void*)  // memory used within the main cache
             + sizeof(int) * this->setHash.size                      // sequence memory
             + (sizeof(void*) + sizeof(BSRouteHash) + sizeof(BSRouteHash)) *
                   this->routes.size()                         // permutations' memory
             + (sizeof(void*)) * this->routes.bucket_count();  // unordered_set internal memory
    }

    inline size_t size() const {
      return routes.size();
    }

    /// hash code of the set
    const BSSetHash setHash;

    /// hash codes of the routes (sequences)
    std::unordered_set<BSRouteHash, BSRouteHash> routes;

    /// original sequence (for debugging purposes only)
    // std::vector<int> originalSequence;

    /// optimized (cached) sequence
    int* sequence;

    /// optimized distance
    double distance;

    /// position in the vector for O(1) removal
    size_t index;

    /// number of times this cache element was used
    size_t usage;

    /**
     * Returns true if b1 has higher usage than b2 and false otherwise
     */
    static bool higherUsage(const BSCache* b1, const BSCache* b2);

    /**
     * Runs tests with the BSCache (to check collisions, for example)
     */
    static int runTest(int argc, char* argv[]);
};

#endif  // BSCache

#endif  // BSCACHE_H
