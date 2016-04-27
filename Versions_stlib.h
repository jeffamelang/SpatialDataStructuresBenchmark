#ifndef VERSIONS_STLIB_H
#define VERSIONS_STLIB_H

#include "CommonDefinitions.h"

#include <vector>

// stlib includes
#include <stlib/geom/orq/Octree.h>
#include <stlib/geom/orq/KDTree.h>
#include <stlib/geom/orq/CellArrayNeighbors.h>

// a little naughty, yeah.
using std::vector;
using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::duration_cast;

void
findNeighbors_stlibCellArray(const vector<Point> & points,
                             const BoundingBox & pointsBoundingBox,
                             const double neighborSearchDistance,
                             vector<unsigned int> * const result,
                             vector<unsigned int> * const resultDelimiters,
                             double * const initializationTime,
                             double * const queryingTime) {
  ignoreUnusedVariable(pointsBoundingBox);

  const double neighborSearchDistanceSquared =
    neighborSearchDistance * neighborSearchDistance;
  const double paddedNeighborSearchDistance = neighborSearchDistance * 1.001;

  stlib::geom::CellArrayNeighbors<double, 3,
                           typename vector<Point>::const_iterator,
                           stlib::ads::Dereference<typename vector<Point>::const_iterator> > orq;

  const high_resolution_clock::time_point initializationTic =
    high_resolution_clock::now();

  orq.initialize(points.begin(), points.end());

  const high_resolution_clock::time_point initializationToc =
    high_resolution_clock::now();

  resultDelimiters->push_back(0);

  const unsigned int numberOfPoints = points.size();
  vector<typename vector<Point>::const_iterator> thisPointsNeighborhood;
  for (unsigned int pointIndex = 0; pointIndex < numberOfPoints; ++pointIndex) {

    const Point thisPoint = points[pointIndex];

    thisPointsNeighborhood.resize(0);

    orq.neighborQuery(points[pointIndex], paddedNeighborSearchDistance,
                      std::back_inserter(thisPointsNeighborhood));

    for (const auto it : thisPointsNeighborhood) {
      if (squaredMagnitude(*it - thisPoint) <=
          neighborSearchDistanceSquared) {
        result->push_back(std::distance(points.cbegin(), it));
      }
    }

    resultDelimiters->push_back(result->size());
  }

  const high_resolution_clock::time_point queryingToc =
    high_resolution_clock::now();

  *initializationTime =
    duration_cast<duration<double> >(initializationToc - initializationTic).count();
  *queryingTime =
    duration_cast<duration<double> >(queryingToc - initializationToc).count();

}

void
findNeighbors_stlibOctTree(const vector<Point> & points,
                           const BoundingBox & pointsBoundingBox,
                           const double neighborSearchDistance,
                           vector<unsigned int> * const result,
                           vector<unsigned int> * const resultDelimiters,
                           double * const initializationTime,
                           double * const queryingTime) {
  const double neighborSearchDistanceSquared =
    neighborSearchDistance * neighborSearchDistance;
  const double paddedNeighborSearchDistance = neighborSearchDistance * 1.001;

  const high_resolution_clock::time_point initializationTic =
    high_resolution_clock::now();

  stlib::geom::Octree<stlib::ads::Dereference<typename vector<Point>::const_iterator> >
    orq(pointsBoundingBox, 8);
  for (typename vector<Point>::const_iterator iterator = points.begin();
       iterator != points.end(); ++iterator) {
    orq.insert(iterator);
  }

  const high_resolution_clock::time_point initializationToc =
    high_resolution_clock::now();

  resultDelimiters->push_back(0);

  const unsigned int numberOfPoints = points.size();
  vector<typename vector<Point>::const_iterator> thisPointsNeighborhood;
  for (unsigned int pointIndex = 0; pointIndex < numberOfPoints; ++pointIndex) {

    const Point thisPoint = points[pointIndex];

    thisPointsNeighborhood.resize(0);

    BoundingBox window;
    window.upper = thisPoint;
    window.lower = window.upper;
    stlib::geom::offset(&window, paddedNeighborSearchDistance);
    thisPointsNeighborhood.resize(0);
    orq.computeWindowQuery(std::back_inserter(thisPointsNeighborhood),
                           window);

    for (const auto it : thisPointsNeighborhood) {
      if (squaredMagnitude(*it - thisPoint) <=
          neighborSearchDistanceSquared) {
        result->push_back(std::distance(points.cbegin(), it));
      }
    }

    resultDelimiters->push_back(result->size());
  }

  const high_resolution_clock::time_point queryingToc =
    high_resolution_clock::now();

  *initializationTime =
    duration_cast<duration<double> >(initializationToc - initializationTic).count();
  *queryingTime =
    duration_cast<duration<double> >(queryingToc - initializationToc).count();

}

void
findNeighbors_stlibKdTree(const vector<Point> & points,
                          const BoundingBox & pointsBoundingBox,
                          const double neighborSearchDistance,
                          vector<unsigned int> * const result,
                          vector<unsigned int> * const resultDelimiters,
                          double * const initializationTime,
                          double * const queryingTime) {
  ignoreUnusedVariable(pointsBoundingBox);

  const double neighborSearchDistanceSquared =
    neighborSearchDistance * neighborSearchDistance;
  const double paddedNeighborSearchDistance = neighborSearchDistance * 1.001;

  const high_resolution_clock::time_point initializationTic =
    high_resolution_clock::now();

  stlib::geom::KDTree<3, stlib::ads::Dereference<typename vector<Point>::const_iterator> >
    orq(points.begin(), points.end());

  const high_resolution_clock::time_point initializationToc =
    high_resolution_clock::now();

  resultDelimiters->push_back(0);

  const unsigned int numberOfPoints = points.size();
  vector<typename vector<Point>::const_iterator> thisPointsNeighborhood;
  for (unsigned int pointIndex = 0; pointIndex < numberOfPoints; ++pointIndex) {

    const Point thisPoint = points[pointIndex];

    thisPointsNeighborhood.resize(0);

    BoundingBox window;
    window.upper = thisPoint;
    window.lower = window.upper;
    stlib::geom::offset(&window, paddedNeighborSearchDistance);
    thisPointsNeighborhood.resize(0);
    orq.computeWindowQuery(std::back_inserter(thisPointsNeighborhood),
                           window);

    for (const auto it : thisPointsNeighborhood) {
      if (squaredMagnitude(*it - thisPoint) <=
          neighborSearchDistanceSquared) {
        result->push_back(std::distance(points.cbegin(), it));
      }
    }

    resultDelimiters->push_back(result->size());
  }

  const high_resolution_clock::time_point queryingToc =
    high_resolution_clock::now();

  *initializationTime =
    duration_cast<duration<double> >(initializationToc - initializationTic).count();
  *queryingTime =
    duration_cast<duration<double> >(queryingToc - initializationToc).count();
}
#endif // DEFINED VERSIONS_STLIB_H

