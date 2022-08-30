#include "application.h"

#include <time.h>

#include <boost/algorithm/string.hpp>
#include <iostream>
#include <string>

using namespace std;

time_t Application::startTime;
int Application::timeLimit;
bool Application::verbose;
std::string Application::version = "v1.0.0";

template <typename T>
inline boost::program_options::typed_value<T>* default_param(T value) {
  return boost::program_options::value<T>()->default_value(value);
}

boost::program_options::variables_map Application::initializeVariablesMap(int argc, char* argv[]) {
  boost::program_options::variables_map variablesMap;
  boost::program_options::options_description description("pdprr");
  boost::program_options::options_description_easy_init add_option = description.add_options();

  add_option("help", "Display a help message.");

  add_option("version", "Display the current version.");

  add_option("verbose", boost::program_options::bool_switch(), "Enable verbose mode.");

  add_option("instance", boost::program_options::value<string>(), "Instance file path.");

  add_option("fast", "Use the fast reinsertion operator");

  add_option("seed", default_param(DEFAULT_SEED), "Sets a random seed.");

  add_option("p-accept", default_param(DEFAULT_P), "p parameter to accept a request as worst.");

  add_option("c-rate", default_param(DEFAULT_C_RATE), "Cooling rate.");

  add_option("it", default_param(DEFAULT_MAX_IT), "Maximum number of iterations.");

  add_option("time-limit", boost::program_options::value<int>(), "Set maximum execution time in seconds.");

  boost::program_options::store(
      boost::program_options::command_line_parser(argc, (const char* const*)argv).options(description).run(),
      variablesMap);

  boost::program_options::notify(variablesMap);

  if (variablesMap.count("help")) {
    std::cout << description << endl;
    exit(0);
  }

  if (variablesMap.count("version")) {
    std::cout << Version() << std::endl;
    exit(0);
  }

  startTime = clock();

  timeLimit = INT_MAX;
  if (variablesMap.count("time-limit") > 0) {
    timeLimit = variablesMap["time-limit"].as<int>();
    Application::SetTimeLimit(timeLimit);
  }

  Application::verbose = variablesMap["verbose"].as<bool>();
  return variablesMap;
}

bool Application::IsVerbose() {
  return verbose;
}

std::string Application::Version() {
  return version;
}

void Application::SetTimeLimit(int timeLimit) {
  Application::timeLimit = timeLimit;
}

double Application::Ellapsed() {
  return static_cast<double>(clock() - startTime) / CLOCKS_PER_SEC;
}

double Application::EllapsedRatio() {
  return Ellapsed() / timeLimit;
}

bool Application::Timeout() {
  double ellapsed = Ellapsed();
  return (ellapsed > MAX_ALLOWED_RUNTIME) || (ellapsed > timeLimit);
}
