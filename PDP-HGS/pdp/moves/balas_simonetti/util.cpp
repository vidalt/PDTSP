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

#include "util.h"

#include <cassert>

/// temporary variable used to store the string to be formatted
char _tmpTextString[1024 * 5] = {'\0'};

/**
 * Returns the number of bits equal to one in a bitset (int), using Brian
 * Kernighan's method (O(k), with k being the number of 1's)
 */
int count_bits(int n) {
  int count = 0;  // count accumulates the total bits set
  while (n != 0) {
    n &= (n - 1);  // clear the least significant bit set
    count++;
  }
  return count;
}

/**
 * Applies "printf-style" formatting while creating a std::string
 */
std::string string_format(const char *format, ...) {
  _tmpTextString[0] = '\0';

  va_list args;
  va_start(args, format);
  int size = vsnprintf(_tmpTextString, 1024 * 5, format, args);
  assert(size <= 1024 * 5);
  va_end(args);
  std::string retStr = _tmpTextString;
  return retStr;
}

/**
 * Checks if a number is prime
 *
 * @param x number to test
 * @returns true if x is prime and false otherwise
 */
bool is_prime(std::size_t x) {
  std::size_t o = 4;
  for (std::size_t i = 5; true; i += o) {
    std::size_t q = x / i;
    if (q < i) return true;
    if (x == q * i) return false;
    o ^= 6;
  }
  return true;
}

/**
 * Returns the next prime number after x
 *
 * @param x considered number
 * @returns the next prime number
 */
std::size_t next_prime(std::size_t x) {
  switch (x) {
    case 0:
    case 1:
    case 2:
      return 2;
    case 3:
      return 3;
    case 4:
    case 5:
      return 5;
  }
  std::size_t k = x / 6;
  std::size_t i = x - 6 * k;
  std::size_t o = i < 2 ? 1 : 5;
  x = 6 * k + o;
  for (i = (3 + o) / 2; !is_prime(x); x += i)
    i ^= 6;
  return x;
}
