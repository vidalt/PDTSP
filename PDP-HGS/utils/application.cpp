#include "application.h"

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <string>

using namespace std;

bool Application::verbose = false;
int Application::time_limit;
int Application::bs_k;
int Application::or_k;

int Application::hgsadc_populationSize;
int Application::hgsadc_maxIterationsWithoutImprovement;
int Application::hgsadc_divIterationsWithoutImprovement;
int Application::hgsadc_offspringInGeneration;
int Application::hgsadc_el;
int Application::hgsadc_cl;

std::string Application::neighborhoods;
double Application::slow_nb_percentage;
bool Application::ls_2kopt = false;
bool Application::ls_relocate = false;
bool Application::ls_2opt = false;
bool Application::ls_4opt_cd = false;
bool Application::ls_4opt_dc = false;
bool Application::ls_4opt_dd = false;
bool Application::ls_bs = false;
bool Application::ls_oropt = false;
bool Application::grubhubmode = false;
bool Application::firstimprovement = false;
std::vector<Application::EvolutionEntry> Application::evolution;
size_t Application::evolutionCount;
double Application::bestCost;

clock_t Application::startTime = 0.0;

std::string Application::instanceFile;
ga::Problem *Application::instance;

template <typename T>
inline boost::program_options::typed_value<T> *default_param(T value) {
  return boost::program_options::value<T>()->default_value(value);
}

boost::program_options::variables_map Application::initializeVariablesMap(int argc, char *argv[]) {
  boost::program_options::variables_map variablesMap;

  boost::program_options::options_description description("pdphgs");
  boost::program_options::options_description_easy_init add_option = description.add_options();

  add_option("help", "Display a help message.");

  add_option("version", "Display the current version.");

  add_option("verbose", boost::program_options::bool_switch(), "Enable verbose mode.");

  add_option("instance", boost::program_options::value<string>(), "Instance file path.");

  add_option("grubhub", "Read file as a distance matrix (GrubHub format).");

  add_option("seed", default_param(DEFAULT_SEED), "Sets a random seed.");

  add_option("time-limit", default_param(INT_MAX), "Set the maximum execution time in seconds.");

  add_option("bs-k", default_param(DEFAULT_BS_K), "Balas&Simonetti k parameter.");

  add_option("or-k", default_param(DEFAULT_OR_K), "Or-Opt k parameter.");

  add_option("ratio-slow-nb", default_param(DEFAULT_SLOW_NB),
             "Ratio of slow neigborhods usage in local searches.");

  add_option("neighborhoods", default_param(DEFAULT_NEIGHBORHOODS), "Select neighborhood structure.");

  add_option("mu", default_param(DEFAULT_POPULATION_SIZE), "Minimum population size.");

  add_option("it", default_param(DEFAULT_MAX_ITERATIONS_WITHOUT_IMPROVEMENT),
             "Maximum iterations without improvement.");

  add_option("div", default_param(DEFAULT_DIVERSIFY_ITERATIONS_WITHOUT_IMPROVEMENT),
             "Iterations without improvement that trigger diversification.");

  add_option("lambda", default_param(DEFAULT_OFFSPRING_IN_GENERATION),
             "Number of offspring in each generation.");

  add_option("nb-elite", default_param(DEFAULT_ELITE), "Number of elite individuals.");

  add_option("nb-close", default_param(DEFAULT_CLOSE), "Number of close individuals.");

  boost::program_options::store(
      boost::program_options::command_line_parser(argc, (const char *const *)argv).options(description).run(),
      variablesMap);

  boost::program_options::notify(variablesMap);

  if (variablesMap.count("help")) {
    std::cout << description << endl;
    exit(0);
  }

  if (variablesMap.count("version")) {
    std::cout << STRINGIZE_VALUE_OF(BUILD_PDP_VERSION_MSG) << std::endl;
    exit(0);
  }

  return variablesMap;
}

void Application::LoadArgs(boost::program_options::variables_map variablesMap) {
  // verbosity
  verbose = variablesMap["verbose"].as<bool>();

  time_limit = variablesMap["time-limit"].as<int>();

  // Instance
  instanceFile = variablesMap["instance"].as<string>();

  grubhubmode = variablesMap.count("grubhub") > 0;

  // HGS
  hgsadc_populationSize = variablesMap["mu"].as<int>();

  hgsadc_maxIterationsWithoutImprovement = variablesMap["it"].as<int>();

  hgsadc_divIterationsWithoutImprovement = variablesMap["div"].as<int>();

  hgsadc_offspringInGeneration = variablesMap["lambda"].as<int>();

  hgsadc_el = variablesMap["nb-elite"].as<int>();

  hgsadc_cl = variablesMap["nb-close"].as<int>();

  // NEIGHBORHOOD
  bs_k = (int)variablesMap["bs-k"].as<int>();
  or_k = (int)variablesMap["or-k"].as<int>();
  slow_nb_percentage = variablesMap["ratio-slow-nb"].as<double>();
  neighborhoods = boost::to_upper_copy<std::string>(variablesMap["neighborhoods"].as<string>());
  ls_relocate = ls_2opt = ls_2kopt = ls_4opt_cd = ls_4opt_dc = ls_4opt_dd = ls_bs = false;

  string buff = neighborhoods.c_str();
  char *strptr = (char *)buff.c_str();
  char *p = strtok(strptr, "-");
  while (p) {
    ls_relocate |= !strcmp(p, "RELOCATE");
    ls_2opt |= !strcmp(p, "2OPT");
    ls_2kopt |= !strcmp(p, "2KOPT");
    ls_oropt |= !strcmp(p, "OROPT");
    ls_4opt_cd |= !strcmp(p, "4OPT");
    ls_4opt_dc |= !strcmp(p, "4OPT");
    ls_4opt_dd |= !strcmp(p, "4OPT");
    ls_bs |= !strcmp(p, "BS");
    p = strtok(0, "-");
  }
}

void Application::PrintParameters() {
  // HGSADC
  cout << "\t  --it=" << hgsadc_maxIterationsWithoutImprovement << endl;
  cout << "\t  --div=" << hgsadc_divIterationsWithoutImprovement << endl;
  cout << "\t  --time-limit=" << time_limit << endl;

  cout << "\t  --mu=" << hgsadc_populationSize << endl;
  cout << "\t  --lambda=" << hgsadc_offspringInGeneration << endl;
  cout << "\t  --nb-elite=" << hgsadc_el << endl;
  cout << "\t  --nb-close=" << hgsadc_cl << endl;

  // NEIGHBORHOOD
  cout << "\t  --neighborhoods=" << neighborhoods << endl;
  cout << "\t  --bs-k=" << bs_k << endl;
  cout << "\t  --or-k=" << or_k << endl;
}

double Application::Ellapsed() {
  return static_cast<double>(clock() - startTime) / CLOCKS_PER_SEC;
}

bool Application::Timeout() {
  return Application::time_limit < Application::Ellapsed();
}

void Application::ClearLogEvolution() {
  evolutionCount = 0;
  bestCost = DBL_MAX;
  evolution.clear();
}
void Application::LogEvolution(double cost) {
  evolutionCount++;
  if (cost < bestCost) {
    evolution.push_back(EvolutionEntry(evolutionCount, Ellapsed(), cost));
    bestCost = cost;
  }
}
