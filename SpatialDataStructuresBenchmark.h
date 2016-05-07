#ifndef SPATIAL_DATA_STRUCTURES_BENCHMARK_H
#define SPATIAL_DATA_STRUCTURES_BENCHMARK_H

#include "CommonDefinitions.h"

#include <string>
#include <vector>
#include <limits>
#include <functional>
#include <utility>

using Function = std::function<
  void(const std::vector<Point>,
       const BoundingBox,
       const double,
       std::vector<unsigned int> * const,
       std::vector<unsigned int> * const,
       double * const,
       double * const)>;

extern std::vector<std::pair<Function, std::string>> functions;

void
checkResult(const std::vector<unsigned int> & correctResult,
            const std::vector<unsigned int> & correctResultDelimiters,
            const std::vector<unsigned int> & testResult,
            const std::vector<unsigned int> & testResultDelimiters,
            const std::string & testName);

template <class Function>
void
runTest(const unsigned int numberOfTrials,
        const Function function,
        const std::vector<Point> & points,
        const BoundingBox & pointsBoundingBox,
        const double neighborSearchDistance,
        std::vector<unsigned int> * const result,
        std::vector<unsigned int> * const resultDelimiters,
        double * const initializationTime,
        double * const queryingTime) {

  *initializationTime = std::numeric_limits<double>::max();
  *queryingTime = std::numeric_limits<double>::max();

  for (unsigned int trialNumber = 0;
       trialNumber < numberOfTrials; ++trialNumber) {

    double thisTrialsInitializationTime = std::numeric_limits<double>::max();
    double thisTrialsQueryingTime = std::numeric_limits<double>::max();

    result->resize(0);
    resultDelimiters->resize(0);

    // Do the test
    function(points, pointsBoundingBox, neighborSearchDistance,
             result, resultDelimiters,
             &thisTrialsInitializationTime, &thisTrialsQueryingTime);

    // Take the minimum values from all trials
    *initializationTime =
      std::min(*initializationTime, thisTrialsInitializationTime);
    *queryingTime =
      std::min(*queryingTime, thisTrialsQueryingTime);
  }

}
#endif // DEFINED SPATIAL_DATA_STRUCTURES_BENCHMARK_H
