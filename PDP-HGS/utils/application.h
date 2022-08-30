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

#define TOSTR_(x...) #x
#define STRINGIZE_VALUE_OF(x) TOSTR_(x)

#define DEFAULT_SEED 0
#define DEFAULT_ELITE 1
#define DEFAULT_CLOSE 2
#define DEFAULT_POPULATION_SIZE 25
#define DEFAULT_OFFSPRING_IN_GENERATION 40
#define DEFAULT_MAX_ITERATIONS_WITHOUT_IMPROVEMENT 1000000
#define DEFAULT_DIVERSIFY_ITERATIONS_WITHOUT_IMPROVEMENT 4000
#define DEFAULT_NEIGHBORHOODS std::string("RELOCATE-2OPT-2KOPT-OROPT-4OPT-BS")

#define DEFAULT_BS_K 3
#define DEFAULT_OR_K 30
#define DEFAULT_SLOW_NB 1.0

#include <float.h>
#include <limits.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "hgsadc/problem.h"

class Application {
  public:
    static boost::program_options::variables_map initializeVariablesMap(int argc, char* argv[]);

    static double Ellapsed();

    static bool Timeout();

    static void LogEvolution(double cost);
    static void ClearLogEvolution();

  public:
    typedef struct _EvolutionEntry {
        size_t iteration;
        double time;
        double cost;

        _EvolutionEntry() : _EvolutionEntry(0, 0, 0) {
        }
        _EvolutionEntry(size_t iteration_, double time_, double cost_)
            : iteration(iteration_), time(time_), cost(cost_) {
        }

    } EvolutionEntry;

    //! Load arguments passed through command line.
    static void LoadArgs(boost::program_options::variables_map variablesMap);

    static void PrintParameters();

    //! Execution time limit in ms.
    static int time_limit;

    //! Flag of verbose mode.
    static bool verbose;

    //! Balas&Simonetti k parameter.
    static int bs_k;

    //! Or-Opt k parameter.
    static int or_k;

    //! Local search neighborhood.
    static std::string neighborhoods;

    //! Percentage of slow neighborhood usage.
    static double slow_nb_percentage;

    //! Local search K2-OPT neighborhood.
    static bool ls_2kopt;

    //! Local search RELOCATE neighborhood.
    static bool ls_relocate;

    //! Local search 2-OPT neighborhood.
    static bool ls_2opt;

    //! Local search 4-OPT-CD neighborhood.
    static bool ls_4opt_cd;

    //! Local search 4-OPT-DC neighborhood.
    static bool ls_4opt_dc;

    //! Local search 4-OPT-DD neighborhood.
    static bool ls_4opt_dd;

    //! Local search  Balas&Simonetti neighborhood.
    static bool ls_bs;

    //! Local search block relocate neighborhood.
    static bool ls_oropt;

    //! Path for instance file.
    static std::string instanceFile;

    //! Problem instance handler.
    static ga::Problem* instance;

    //! Hybrid Genetic Search with Advanced Diversity Control population size.
    static int hgsadc_populationSize;

    //! Hybrid Genetic Search with Advanced Diversity Control maximum iterations
    //! without improvement.
    static int hgsadc_maxIterationsWithoutImprovement;

    //! Hybrid Genetic Search with Advanced Diversity Control diversify after n
    //! iterations without improvement.
    static int hgsadc_divIterationsWithoutImprovement;

    //! Hybrid Genetic Search with Advanced Diversity Control Offsping In a
    //! Generation.
    static int hgsadc_offspringInGeneration;

    //! Hybrid Genetic Search with Advanced Diversity Control Number of Elite
    //! Individuals.
    static int hgsadc_el;

    //! Hybrid Genetic Search with Advanced Diversity Control Number of Close
    //! Individuals.
    static int hgsadc_cl;

    //! Evolutions log
    static std::vector<EvolutionEntry> evolution;
    static size_t evolutionCount;

    //! Current best cost
    static double bestCost;

    //! Set instance as edge mode (grubhub)
    static bool grubhubmode;

    //! Set instance as edge mode (grubhub)
    static bool firstimprovement;

    static clock_t startTime;
};

#endif  // APPLICATION_H
