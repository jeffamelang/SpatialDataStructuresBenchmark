// -*- C++ -*-
#ifndef VERSIONS_PCL_H
#define VERSIONS_PCL_H

// pcl includes
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

// a little naughty, yeah.
using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::duration_cast;

void
findNeighbors_pclOctree(const vector<Point> & points,
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Generate pointcloud
    cloud->points.resize(points.size());

    for (size_t i=0; i < points.size(); ++i) {
        cloud->points[i].x = points[i][0];
        cloud->points[i].y = points[i][1];
        cloud->points[i].z = points[i][2];
    }

    // set octree resolution. using given number from pcl octree example
    float resolution = 0.05f;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud ();

    const high_resolution_clock::time_point initializationToc =
        high_resolution_clock::now();

    resultDelimiters->push_back(0);

    const unsigned int numberOfPoints = points.size();
    vector<typename vector<Point>::const_iterator> thisPointsNeighborhood;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    for (unsigned int pointIndex = 0; pointIndex < numberOfPoints; ++pointIndex) {
        pcl::PointXYZ thisPoint;
        thisPoint.x = points[pointIndex][0];
        thisPoint.y = points[pointIndex][1];
        thisPoint.z = points[pointIndex][2];

        pointIdxRadiusSearch.resize(0);
        pointRadiusSquaredDistance.resize(0);
        octree.radiusSearch( thisPoint, paddedNeighborSearchDistance,
            pointIdxRadiusSearch, pointRadiusSquaredDistance);

        for (const auto it : pointIdxRadiusSearch) {
            if (squaredMagnitude(points[it] - points[pointIndex]) <=
                    neighborSearchDistanceSquared) {
                result->push_back(it);
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
findNeighbors_pclKdTree(const vector<Point> & points,
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Generate pointcloud
    cloud->points.resize(points.size());

    for (size_t i=0; i < points.size(); ++i) {
        cloud->points[i].x = points[i][0];
        cloud->points[i].y = points[i][1];
        cloud->points[i].z = points[i][2];
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    const high_resolution_clock::time_point initializationToc =
        high_resolution_clock::now();

    resultDelimiters->push_back(0);

    const unsigned int numberOfPoints = points.size();
    vector<typename vector<Point>::const_iterator> thisPointsNeighborhood;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    for (unsigned int pointIndex = 0; pointIndex < numberOfPoints; ++pointIndex) {
        pcl::PointXYZ thisPoint;
        thisPoint.x = points[pointIndex][0];
        thisPoint.y = points[pointIndex][1];
        thisPoint.z = points[pointIndex][2];

        pointIdxRadiusSearch.resize(0);
        pointRadiusSquaredDistance.resize(0);
        kdtree.radiusSearch( thisPoint, paddedNeighborSearchDistance,
            pointIdxRadiusSearch, pointRadiusSquaredDistance);

        for (const auto it : pointIdxRadiusSearch) {
            if (squaredMagnitude(points[it] - points[pointIndex]) <=
                    neighborSearchDistanceSquared) {
                result->push_back(it);
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

#endif // VERSIONS_PCL_H
