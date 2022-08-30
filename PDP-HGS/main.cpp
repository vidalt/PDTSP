#include <limits.h>

#include <iostream>

#include "hgsadc/hgsadc.h"
#include "pdp/pdpinstance.h"
#include "utils/application.h"
#include "utils/random.h"

using namespace pdp;
using namespace std;

/* running parameters example:
--seed=123456
--neighborhoods=RELOCATE-2OPT-OROPT-2KOPT-4OPT-BS
--hgsadc-max-iterations-without-improvement=1000
--hgsadc-close=2
--hgsadc-elite=1
--or-k=7
--bs-k=5
--instance=instances/RBO00/Class1/U159C.PDT
--verbose
*/

int main(int argc, char* argv[]) {
  boost::program_options::variables_map variablesMap = Application::initializeVariablesMap(argc, argv);

  uint seed = (uint)variablesMap["seed"].as<int>();
  std::srand(seed);

  // solve only one instance specified by the --instance cmdline argument
  if (variablesMap["instance"].empty()) {
    cout << "No input instance. Use --help for more information" << endl;
    exit(1);
  }

  Application::LoadArgs(variablesMap);

  // load instance file
  Application::instance = PDPInstance::fromFilePath(Application::instanceFile);
  Application::instance->Precompute();
  Application::startTime = clock();
  Application::evolution.clear();
  Application::bestCost = FLT_MAX;

  if (Application::verbose) {
    std::cout << "RUN LOG START:" << endl << endl;

    std::cout << "\t" << STRINGIZE_VALUE_OF(BUILD_PDP_VERSION) << endl;
    std::cout << "\t" << STRINGIZE_VALUE_OF(BUILD_PDP_VERSION_MSG) << endl << endl;

    if (system(" date \"+\tDATE: %d/%m/%y %H:%M:%S\"") != 0) {
      std::cout << "\tDATE: N/A";
      std::cout << endl;
    }

    std::cout << "\t";
    std::cout.flush();
    if (system("cat /proc/meminfo | grep 'MemTotal:'") != 0) {
      std::cout << "MemTotal: N/A";
      std::cout << endl;
    }

    std::cout << "\tCPU ";
    std::cout.flush();
    if (system("cat /proc/cpuinfo | grep 'model name' -m1") != 0) {
      std::cout << "model name N/A";
      std::cout << endl;
    }

    std::cout << endl << endl << "PARAMETERS LOG:" << endl << endl;
    std::cout << "\tSEED: " << seed << endl;
    std::cout << "\tNEIGHBORHOOD: " << Application::neighborhoods << endl;
    std::cout << "\tINSTANCE FILE: " << boost::filesystem::system_complete(Application::instanceFile) << endl;

    std::cout << endl << "\tCMD: " << endl;
    for (int i = 1; i < argc; i++) {
      std::cout << "\t  " << argv[i] << endl;
    }

    cout << endl << "\tALL:" << endl;
    Application::PrintParameters();

    std::cout << endl << endl << "SEARCH LOG:" << endl << endl;
  }

  ga::Solution* finalSolution = Application::instance->CreateEmptySolution();
  ga::HGSADC::Solve(finalSolution, *Application::instance);  // execute solver

  if (Application::verbose) {
    std::cout << endl << endl << "SEARCH STATS:" << endl << endl;
    std::cout << "\tCOST: " << finalSolution->Cost() << endl;
    std::cout << "\tTIME: " << Application::Ellapsed() << "s" << endl;
    std::cout << "\tLOCAL SEARCH: " << Application::instance->LSCompleteLog() << endl;
    std::cout << endl << endl << "SOLUTION LOG:" << endl << endl;

    std::cout << "\tSTATUS: " << (finalSolution->IsFeasible() ? "FEASIBLE" : "INFEASIBLE") << endl;

    finalSolution->Print();
    cout << endl << endl << endl;
  } else {
    std::cout << "{" << endl;
    std::cout << "  \"version\": \"" << STRINGIZE_VALUE_OF(BUILD_PDP_VERSION) << "\"," << endl;
    std::cout << "  \"cost\": " << finalSolution->Cost() << "," << endl;
    std::cout << "  \"time\": " << Application::Ellapsed() << "," << endl;
    std::string ls_log = Application::instance->LSCompleteLog();
    std::replace(ls_log.begin(), ls_log.end(), '\t', '\n');
    std::cout << ls_log << "," << endl;

    finalSolution->Print();
    std::cout << "," << endl;
    std::cout << "  \"evolution\": [" << endl;
    std::vector<Application::EvolutionEntry>::iterator evolIt = Application::evolution.begin();
    for (size_t i = 0; i < Application::evolution.size() - 1; i++) {
      Application::EvolutionEntry& entry = *evolIt;
      std::cout << "    {\n";
      std::cout << "       \"iteration\": " << entry.iteration << ",\n";
      std::cout << "       \"time\": " << entry.time << ",\n";
      std::cout << "       \"cost\": " << entry.cost << "\n";
      std::cout << "    },\n";
      evolIt++;
    }
    Application::EvolutionEntry& entry = *evolIt;
    std::cout << "    {\n";
    std::cout << "       \"iteration\": " << entry.iteration << ",\n";
    std::cout << "       \"time\": " << entry.time << ",\n";
    std::cout << "       \"cost\": " << entry.cost << "\n";
    std::cout << "    }\n";
    std::cout << "  ]" << endl;

    std::cout << "}" << endl;
  }

  delete Application::instance;
  delete finalSolution;
  return 0;
}
