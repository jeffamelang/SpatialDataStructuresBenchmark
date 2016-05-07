// -*- C++ -*-
// SpatialDataStructuresBenchmark.cc
// An example to compare the efficiency of different spatial data structures

#include "SpatialDataStructuresBenchmark.h"

#include <string>
#include <vector>
#include <array>
#include <chrono>
#include <tuple>

using std::string;
using std::vector;
using std::array;
using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::duration_cast;

// This file contains the test scenario generators which form the distributions
//  of points.
#include "PointScenarioGenerators.h"

#include "Versions_stlib.h"
#include "Versions_pcl.h"
#if 0
#include "Versions_kdtree2.h"
#endif

vector<std::pair<Function, std::string>> functions = {
  std::make_pair(findNeighbors_stlibCellArray, "stlib_cellarray"),
  std::make_pair(findNeighbors_stlibOctTree, "stlib_octree"),
  std::make_pair(findNeighbors_stlibKdTree, "stlib_kdtree"),
  std::make_pair(findNeighbors_pclKdTree, "pcl_kdtree"),
  std::make_pair(findNeighbors_pclOctree, "pcl_octree"),
  #if 0
  std::make_pair(findNeighbors_kdtree2, "kdtree2")
  #endif
};

// This is a result checking function to make sure we have the right answer.
void
checkResult(const vector<unsigned int> & correctResult,
            const vector<unsigned int> & correctResultDelimiters,
            const vector<unsigned int> & testResult,
            const vector<unsigned int> & testResultDelimiters,
            const std::string & testName) {
  vector<unsigned int> sortedCopyOfCorrectResult = correctResult;
  vector<unsigned int> sortedCopyOfTestResult = testResult;
  char sprintfBuffer[500];
  if (correctResultDelimiters.size() != testResultDelimiters.size()) {
    sprintf(sprintfBuffer, "wrong result, "
            "delimiter sizes don't match: correct is %zu, test is %zu, "
            "test named " BOLD_ON FG_RED "%s" RESET "\n",
            correctResultDelimiters.size(), testResultDelimiters.size(),
            testName.c_str());
    throw std::runtime_error(sprintfBuffer);
  }
  const unsigned int numberOfPoints = correctResultDelimiters.size() - 1;
  for (unsigned int pointIndex = 0;
       pointIndex < numberOfPoints; ++pointIndex) {
    try {
      const unsigned int beginIndex = correctResultDelimiters[pointIndex];
      const unsigned int endIndex = correctResultDelimiters[pointIndex + 1];
      if (testResultDelimiters[pointIndex] != beginIndex) {
        sprintf(sprintfBuffer,
                "beginIndex for point %u doesn't match: correct is %u, "
                "test is %u\n",
                pointIndex, beginIndex, testResultDelimiters[pointIndex]);
        throw std::runtime_error(sprintfBuffer);
      }
      if (testResultDelimiters[pointIndex + 1] != endIndex) {
        sprintf(sprintfBuffer,
                "endIndex for point %u doesn't match: correct is %u, "
                "test is %u\n",
                pointIndex, endIndex, testResultDelimiters[pointIndex + 1]);
        throw std::runtime_error(sprintfBuffer);
      }

      // sort the two neighborhoods
      std::sort(sortedCopyOfCorrectResult.begin() + beginIndex,
                sortedCopyOfCorrectResult.begin() + endIndex);
      std::sort(sortedCopyOfTestResult.begin() + beginIndex,
                sortedCopyOfTestResult.begin() + endIndex);
      // check if the two neighborhoods are the same
      bool neighborhoodsAreTheSame = true;
      for (unsigned int index = beginIndex; index < endIndex; ++index) {
        if (sortedCopyOfCorrectResult[index] != sortedCopyOfTestResult[index]) {
          neighborhoodsAreTheSame = false;
          fprintf(stderr, "point %7u neighbor %3u is different: correct %7u "
                  "test %5u\n",
                  pointIndex, index - beginIndex,
                  sortedCopyOfCorrectResult[index],
                  sortedCopyOfTestResult[index]);
        }
      }
      if (neighborhoodsAreTheSame == false) {
        sprintf(sprintfBuffer, "wrong result, "
                "point %u's neighborhood doesn't match\n",
                pointIndex);
        throw std::runtime_error(sprintfBuffer);
      }
    } catch (const std::exception & e) {
      fprintf(stderr, "wrong result, "
              "point %u had an error, "
              "test named " BOLD_ON FG_RED "%s" RESET "\n",
              pointIndex, testName.c_str());
      throw;
    }
  }
}


int main(int argc, char* argv[]) {
  ignoreUnusedVariable(argc);
  ignoreUnusedVariable(argv);

  // ===========================================================================
  // *************************** < Inputs> *************************************
  // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

  const array<double, 2> numberOfPointsRange = {{1e3, 1e5}};
  const unsigned int numberOfDataPoints      = 10;
  const unsigned int numberOfTrialsPerSize   = 3;
  const double neighborSearchDistance        = 1.0;
  typedef PointScenarioGenerators::UniformRandomWithAverageNumberOfNeighbors PointGenerator;

  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // *************************** </Inputs> *************************************
  // ===========================================================================

  char sprintfBuffer[500];

  const string prefix = "data/SpatialDataStructuresBenchmark_";
  const string suffix = "_knuth";

  // Make sure that the data directory exists.
  Utilities::verifyThatDirectoryExists("data");

  vector<std::tuple<string, Function, FILE*>> versions;
  
  for (const auto& item : functions) {
    sprintf(sprintfBuffer, "%s%s_%s_results%s.csv", 
            prefix.c_str(), PointGenerator::getName().c_str(), 
            std::get<1>(item).c_str(), suffix.c_str());
    versions.push_back(
      std::make_tuple(
        std::get<1>(item), 
        std::get<0>(item), 
        fopen(sprintfBuffer, "w")));
  }

  // For each size
  for (unsigned int dataPointIndex = 0;
       dataPointIndex < numberOfDataPoints;
       ++dataPointIndex) {

    const unsigned int numberOfPoints =
      Utilities::interpolateNumberLinearlyOnLogScale(numberOfPointsRange[0],
                                                     numberOfPointsRange[1],
                                                     numberOfDataPoints,
                                                     dataPointIndex);

    const high_resolution_clock::time_point thisSizesTic =
      high_resolution_clock::now();

    // generate points
    vector<Point> points;

    const unsigned int averageNumberOfNeighborsPerPoint = 70;
    PointGenerator::generatePoints(numberOfPoints,
                                   averageNumberOfNeighborsPerPoint,
                                   neighborSearchDistance, &points);

    // form the bounding box
    BoundingBox temp;
    for (const Point & p : points) {
      temp += p;
    }
    stlib::geom::offset(&temp, 1e-3*(temp.upper[0] - temp.lower[0]));
    const BoundingBox pointsBoundingBox = temp;

    vector<unsigned int> result;
    vector<unsigned int> resultDelimiters;
    double initializationTime;
    double queryingTime;

    // Get the correct result from the cell array
    runTest(numberOfTrialsPerSize,
            std::get<1>(versions[0]),
            points,
            pointsBoundingBox,
            neighborSearchDistance,
            &result,
            &resultDelimiters,
            &initializationTime,
            &queryingTime);
    const vector<unsigned int> correctResult           = result;
    const vector<unsigned int> correctResultDelimiters = resultDelimiters;
    
    // Test for this function's result
    for (const auto& function : versions) {
      runTest(numberOfTrialsPerSize,
              std::get<1>(function),
              points,
              pointsBoundingBox,
              neighborSearchDistance,
              &result,
              &resultDelimiters,
              &initializationTime,
              &queryingTime);
      checkResult(correctResult,
                  correctResultDelimiters,
                  result,
                  resultDelimiters,
                  std::get<0>(function));
      fprintf(std::get<2>(function), "%10.4e, %10.4e, %10.4e\n", 
              static_cast<double>(numberOfPoints), initializationTime, queryingTime);

    }

    const high_resolution_clock::time_point thisSizesToc =
      high_resolution_clock::now();
    const double thisSizesElapsedTime =
      duration_cast<duration<double> >(thisSizesToc - thisSizesTic).count();
    printf("finished %8.2e points in %6.2f seconds\n", double(numberOfPoints),
           thisSizesElapsedTime);

  }

  for (const auto& function : versions) {
    fclose(std::get<2>(function));
  }

  return 0;
}
