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

#ifndef PDPROUTEINFO_H
#define PDPROUTEINFO_H

namespace pdp {

class PDPRouteInfo {
  public:
    //! Default constructor.
    PDPRouteInfo();

    const PDPRouteInfo& operator=(const PDPRouteInfo& c);

    //! Single customer constructor.
    PDPRouteInfo(int singleItem);

    //!  subroute distance
    double distance;

    //!  id of the first customer of route
    int begin;

    //!  id of the last customer of route
    int end;

    //!  number of nodes in route
    int size;

    //!  flag: true if information is valid
    bool valid;
};

}  // namespace pdp

#endif  // PDPROUTEINFO_H
