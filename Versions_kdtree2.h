// -*- C++ -*-
#ifndef VERSIONS_KDTREE2_H
#define VERSIONS_KDTREE2_H

#include "CommonDefinitions.h"

// kdtree2 include
#include "kdtree2.hpp"

// a little naughty, yeah.
using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::duration_cast;


void
findNeighbors_kdtree2(const vector<Point> & points,
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

  const unsigned int numberOfPoints = points.size();

  boost::multi_array<float, 2> data(boost::extents[numberOfPoints][3]);

  for (unsigned int pointIndex = 0; pointIndex < numberOfPoints; ++pointIndex) {
    for (unsigned int coordinate = 0; coordinate < 3; ++coordinate) {
      data[pointIndex][coordinate] = points[pointIndex][coordinate];
    }
  }

  const bool kdtree2ShouldRearrangeTheData = false;
  kdtree2::KDTree * tree =
    new kdtree2::KDTree(data, kdtree2ShouldRearrangeTheData);
  tree->sort_results = true;

  const high_resolution_clock::time_point initializationToc =
    high_resolution_clock::now();

  resultDelimiters->push_back(0);

  std::vector<float> searchLocation(3);
  kdtree2::KDTreeResultVector kdtree2Results;
  for (unsigned int pointIndex = 0; pointIndex < numberOfPoints; ++pointIndex) {

    const Point thisPoint = points[pointIndex];
    for (unsigned int coordinate = 0; coordinate < 3; ++coordinate) {
      searchLocation[coordinate] = thisPoint[coordinate];
    }

    kdtree2Results.resize(0);
    tree->r_nearest(searchLocation, paddedNeighborSearchDistance, kdtree2Results);

    for (const auto it : kdtree2Results) {
      if (squaredMagnitude(points[it.idx] - thisPoint) <=
          neighborSearchDistanceSquared) {
        result->push_back(it.idx);
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

#endif // VERSIONS_KDTREE2_H
