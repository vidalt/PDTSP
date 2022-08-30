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

#ifndef UTIL_H
#define UTIL_H

#include <stdarg.h>

#include <string>

//#define BIG_PRIME 2147483647
#define BIG_PRIME 4294967295
#define EPS 1e-6
#define INF 1e31

std::string string_format(const char* format, ...);

/**
 * Returns the number of bits equal to one in a bit-set (int), using Brian
 * Kernighan's approach
 */
int count_bits(int n);

/**
 * Simple struct to apply less<> operations to containers of pointers
 */
template <class T>
struct lessptr {
    inline bool operator()(const T& x, const T& y) const {
      return (*x) < (*y);
    }
};

/**
 * Simple struct for straight size_t -> hash transformation
 */
struct key_as_hash {
    std::size_t operator()(const size_t& k) const {
      return k;
    }
};

/**
 * Find last (i.e. most significant) set bit
 *
 * @param x value
 * @returns Most significant bit set in value (LSB=1), or zero
 */
static inline int fls(unsigned long long x) {
  int r = 0;

  if (x & 0xffffffff00000000ULL) {
    x >>= 32;
    r += 32;
  }
  if (x & 0xffff0000UL) {
    x >>= 16;
    r += 16;
  }
  if (x & 0xff00) {
    x >>= 8;
    r += 8;
  }
  if (x & 0xf0) {
    x >>= 4;
    r += 4;
  }
  if (x & 0xc) {
    x >>= 2;
    r += 2;
  }
  if (x & 0x2) {
    x >>= 1;
    r += 1;
  }
  return (x ? (r + 1) : 0);
}

/**
 * Checks if a number is prime
 *
 * @param x number to test
 * @returns true if x is prime and false otherwise
 */
bool is_prime(std::size_t x);

/**
 * Returns the next prime number after x
 *
 * @param x considered number
 * @returns the next prime number
 */
std::size_t next_prime(std::size_t x);

#endif  // UTIL_H
