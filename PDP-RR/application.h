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

#ifndef APPLICATION_H
#define APPLICATION_H

#include <limits.h>

#include <boost/program_options.hpp>

#define DEFAULT_SEED 0
#define DEFAULT_P 3.0
#define DEFAULT_C_RATE 0.999875716
#define DEFAULT_MAX_IT 50000

// Max allowed runtime set to 5 hours.
#define MAX_ALLOWED_RUNTIME 18000

class Application {
  public:
    static boost::program_options::variables_map initializeVariablesMap(int argc, char *argv[]);

    static bool UsingTimeLimit() {
      return timeLimit != INT_MAX;
    }

    static double EllapsedRatio();
    static double Ellapsed();
    static std::string Version();
    static bool IsVerbose();
    static bool Timeout();
    static void SetTimeLimit(int timeLimit);

  private:
    static std::string version;
    static time_t startTime;
    static int timeLimit;
    static bool verbose;
};

#endif  // APPLICATION_H
