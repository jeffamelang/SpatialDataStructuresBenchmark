// -*- C++ -*-
// SpatialDataStructuresBenchmark.cc
// An example to compare the efficiency of different spatial data structures

#include "CommonDefinitions.h"

using std::string;
using std::vector;
using std::array;
using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::duration_cast;

// These files contain the functions which run the different spatial data
//  structures.
#include "Versions_stlib.h"
#include "Versions_pcl.h"
#include "Versions_kdtree2.h"

// This file contains the test scenario generators which form the distributions
//  of points.
#include "PointScenarioGenerators.h"

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

template <class Function>
void
runTest(const unsigned int numberOfTrials,
        const Function function,
        const vector<Point> & points,
        const BoundingBox & pointsBoundingBox,
        const double neighborSearchDistance,
        vector<unsigned int> * const result,
        vector<unsigned int> * const resultDelimiters,
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

  // Open output files
  sprintf(sprintfBuffer, "%s%s_stlib_cellArray_results%s.csv",
          prefix.c_str(), PointGenerator::getName().c_str(), suffix.c_str());
  FILE * stlibCellArrayFile = fopen(sprintfBuffer, "w");
  sprintf(sprintfBuffer, "%s%s_stlib_octTree_results%s.csv",
          prefix.c_str(), PointGenerator::getName().c_str(), suffix.c_str());
  FILE * stlibOctTreeFile = fopen(sprintfBuffer, "w");
  sprintf(sprintfBuffer, "%s%s_stlib_kdTree_results%s.csv",
          prefix.c_str(), PointGenerator::getName().c_str(), suffix.c_str());
  FILE * stlibKdTreeFile = fopen(sprintfBuffer, "w");
  sprintf(sprintfBuffer, "%s%s_pcl_kdTree_results%s.csv",
          prefix.c_str(), PointGenerator::getName().c_str(), suffix.c_str());
  FILE * pclKdTreeFile = fopen(sprintfBuffer, "w");
  sprintf(sprintfBuffer, "%s%s_pcl_ocTree_results%s.csv",
          prefix.c_str(), PointGenerator::getName().c_str(), suffix.c_str());
  FILE * pclOctreeFile = fopen(sprintfBuffer, "w");
  sprintf(sprintfBuffer, "%s%s_kdtree2_results%s.csv",
          prefix.c_str(), PointGenerator::getName().c_str(), suffix.c_str());
  FILE * kdtree2File = fopen(sprintfBuffer, "w");

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

    // =========================================================================
    // ********************** < do stlib cellArray > ***************************
    // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

    runTest(numberOfTrialsPerSize,
            findNeighbors_stlibCellArray,
            points,
            pointsBoundingBox,
            neighborSearchDistance,
            &result,
            &resultDelimiters,
            &initializationTime,
            &queryingTime);
    const vector<unsigned int> correctResult           = result;
    const vector<unsigned int> correctResultDelimiters = resultDelimiters;
    fprintf(stlibCellArrayFile, "%10.4e, %10.4e, %10.4e\n",
            double(numberOfPoints), initializationTime, queryingTime);

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // ********************** < do stlib cellArray > ***************************
    // =========================================================================

    // =========================================================================
    // ********************** < do stlib octTree > ***************************
    // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

    runTest(numberOfTrialsPerSize,
            findNeighbors_stlibOctTree,
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
                string("stlib octTree"));
    fprintf(stlibOctTreeFile, "%10.4e, %10.4e, %10.4e\n",
            double(numberOfPoints), initializationTime, queryingTime);

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // ********************** < do stlib octTree > ***************************
    // =========================================================================

    // =========================================================================
    // ********************** < do stlib kdTree > ***************************
    // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

    runTest(numberOfTrialsPerSize,
            findNeighbors_stlibKdTree,
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
                string("stlib kdTree"));
    fprintf(stlibKdTreeFile, "%10.4e, %10.4e, %10.4e\n",
            double(numberOfPoints), initializationTime, queryingTime);

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // ********************** < do stlib kdTree > ***************************
    // =========================================================================

    // =========================================================================
    // ********************** < do pcl kdTree > ***************************
    // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

    runTest(numberOfTrialsPerSize,
            findNeighbors_pclKdTree,
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
                string("pcl kdTree"));
    fprintf(pclKdTreeFile, "%10.4e, %10.4e, %10.4e\n",
            double(numberOfPoints), initializationTime, queryingTime);

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // ********************** < do pcl kdTree > ***************************
    // =========================================================================

    // =========================================================================
    // ********************** < do pcl octree > ***************************
    // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

    runTest(numberOfTrialsPerSize,
            findNeighbors_pclOctree,
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
                string("pcl octree"));
    fprintf(pclOctreeFile, "%10.4e, %10.4e, %10.4e\n",
            double(numberOfPoints), initializationTime, queryingTime);

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // ********************** < do pcl octree > ***************************
    // =========================================================================

    // =========================================================================
    // ********************** < do kdtree2 > ***************************
    // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

    runTest(numberOfTrialsPerSize,
            findNeighbors_kdtree2,
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
                string("kdtree2"));
    fprintf(kdtree2File, "%10.4e, %10.4e, %10.4e\n",
            double(numberOfPoints), initializationTime, queryingTime);

    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // ********************** < do kdtree2 > ***************************
    // =========================================================================

    const high_resolution_clock::time_point thisSizesToc =
      high_resolution_clock::now();
    const double thisSizesElapsedTime =
      duration_cast<duration<double> >(thisSizesToc - thisSizesTic).count();
    printf("finished %8.2e points in %6.2f seconds\n", double(numberOfPoints),
           thisSizesElapsedTime);

  }

  fclose(stlibCellArrayFile);
  fclose(stlibOctTreeFile);
  fclose(stlibKdTreeFile);
  fclose(pclKdTreeFile);
  fclose(pclOctreeFile);
  fclose(kdtree2File);

  return 0;
}
