// -*- C++ -*-
#ifndef POINT_SCENARIO_GENERATORS_H
#define POINT_SCENARIO_GENERATORS_H

#include "CommonDefinitions.h"

namespace PointScenarioGenerators {

struct UniformRandomWithAverageNumberOfNeighbors {

  static
  void
  generatePoints(const unsigned int numberOfPoints,
                 const unsigned int averageNumberOfNeighborsPerPoint,
                 const double neighborSearchDistance,
                 vector<Point> * points) {
    const double cellVolume = pow(2 * neighborSearchDistance, 3);
    const double density = averageNumberOfNeighborsPerPoint / cellVolume;
    const double totalVolume = numberOfPoints / density;
    const double domainSideLength = pow(totalVolume, 1./3);

    std::default_random_engine randomNumberEngine;
    std::uniform_real_distribution<double> randomNumberGenerator(0, domainSideLength);

    for (unsigned int pointIndex = 0;
         pointIndex < numberOfPoints; ++pointIndex) {
      points->push_back((Point)
                        {{randomNumberGenerator(randomNumberEngine),
                              randomNumberGenerator(randomNumberEngine),
                              randomNumberGenerator(randomNumberEngine)}});
    }
  }

  static
  std::string
  getName() {
    return std::string("uniformRandomWithAverageNumberOfNeighbors");
  }

};

struct NonUniformRandomWithAverageNumberOfNeighbors {

  static
  void
  generatePoints(const unsigned int numberOfPoints,
                 const unsigned int averageNumberOfNeighborsPerPoint,
                 const unsigned int numGroups,
                 const double neighborSearchDistance,
                 vector<Point> * points) {
    const double cellVolume = pow(2 * neighborSearchDistance, 3);
    const double density = averageNumberOfNeighborsPerPoint / cellVolume;
    const double totalVolume = numberOfPoints / density;
    const double domainSideLength = pow(totalVolume, 1./3);

    std::default_random_engine randomNumberEngine;
    std::vector<std::normal_distribution<double> > normalRandL(numGroups);
    std::uniform_real_distribution<double> uniformRandGen(0, domainSideLength);
    std::uniform_real_distribution<double> distPick(0, numGroups);

    for(unsigned int i = 0; i < numGroups; ++i){
      std::normal_distribution<double> distribution(uniformRandGen(randomNumberEngine),
                                                    uniformRandGen(randomNumberEngine));
      normalRandL[i] = distribution;
    }

    for (unsigned int pointIndex = 0;
         pointIndex < numberOfPoints; ++pointIndex) {
      int x = (int)distPick(randomNumberEngine);
      int y = (int)distPick(randomNumberEngine);
      int z = (int)distPick(randomNumberEngine);

      points->push_back((Point)
                        {{std::fmod(normalRandL[x](randomNumberEngine),domainSideLength),
                          std::fmod(normalRandL[y](randomNumberEngine),domainSideLength),
                          std::fmod(normalRandL[z](randomNumberEngine),domainSideLength)}});
    }
  }

  static
  std::string
  getName() {
    return std::string("nonuniformRandomWithAverageNumberOfNeighbors");
  }

};

struct GaussianDistribution {

  static
  void
  generatePoints(const unsigned int numberOfPoints,
                 const unsigned int averageNumberOfNeighborsPerPoint,
                 const double neighborSearchDistance,
                 vector<Point> * points) {
    ignoreUnusedVariable(averageNumberOfNeighborsPerPoint);

    std::default_random_engine randomNumberEngine;
    std::normal_distribution<> randomNumberGenerator(0, neighborSearchDistance);
    for (unsigned int pointIndex = 0;
         pointIndex < numberOfPoints; ++pointIndex) {
      points->push_back((Point)
                        {{randomNumberGenerator(randomNumberEngine),
                              randomNumberGenerator(randomNumberEngine),
                              randomNumberGenerator(randomNumberEngine)}});
    }
  }

  static
  std::string
  getName() {
    return std::string("gaussianDistribution");
  }

};
}

#endif // POINT_SCENARIO_GENERATORS_H
