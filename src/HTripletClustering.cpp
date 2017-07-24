//
// Created by Joshua Bradt on 7/24/17.
//

#include "HTripletClustering.h"

namespace hc {
    void HTripletClustering::generateSmoothedCloud()
    {
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

        kdtree.setInputCloud(xyziCloud);

        for(size_t i = 0; i < xyziCloud->size(); ++i)
        {
            pcl::PointXYZI newPoint;

            pcl::CentroidPoint<pcl::PointXYZI> centroid;
            pcl::PointXYZI centroidPoint;
            std::vector<int> pointIdxNKNSearch;
            std::vector<float> pointNKNSquaredDistance;
            int found = kdtree.radiusSearch(*xyziCloud, (int)i, smoothRadius, pointIdxNKNSearch, pointNKNSquaredDistance);

            for(int j = 0; j < found; ++j) {
                centroid.add((*xyziCloud)[pointIdxNKNSearch[j]]);
            }

            centroid.get(centroidPoint);

            if(smoothUsingMedian)
            {
                std::vector<float> xList;
                std::vector<float> yList;
                std::vector<float> zList;

                for(int j = 0; j < found; ++j) {
                    pcl::PointXYZI const &point = (*xyziCloud)[pointIdxNKNSearch[j]];

                    xList.push_back(point.x);
                    yList.push_back(point.y);
                    zList.push_back(point.z);
                }

                auto xMedianIt = xList.begin() + (xList.size() / 2);
                std::nth_element(xList.begin(), xMedianIt, xList.end());
                newPoint.x = *xMedianIt;

                auto yMedianIt = yList.begin() + (yList.size() / 2);
                std::nth_element(yList.begin(), yMedianIt, yList.end());
                newPoint.y = *yMedianIt;

                auto zMedianIt = zList.begin() + (zList.size() / 2);
                std::nth_element(zList.begin(), zMedianIt, zList.end());
                newPoint.z = *zMedianIt;
            }
            else
                newPoint = centroidPoint;

            smoothCloud->push_back(newPoint);
        }
    }

    void HTripletClustering::generateTriplets()
    {
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(xyziCloud);

        for(size_t pointIndexB = 0; pointIndexB < xyziCloud->size(); ++pointIndexB)
        {
            auto const &pointB = (*xyziCloud)[pointIndexB];
            Eigen::Vector3f pointBEigen(pointB.x, pointB.y, pointB.z);

            std::vector<Triplet> tripletCandidates;

            std::vector<int> nnIndices;
            nnIndices.reserve(genTripletsNnCandidates);
            std::vector<float> nnbSquaredDistances;
            nnbSquaredDistances.reserve(genTripletsNnCandidates);
            int const nnFound = kdtree.nearestKSearch(*xyziCloud, (int)pointIndexB, (int)genTripletsNnCandidates, nnIndices, nnbSquaredDistances);

            for(size_t pointIndexIndexA = 0; pointIndexIndexA < nnFound; ++pointIndexIndexA)
            {
                size_t const pointIndexA = nnIndices[pointIndexIndexA];
                auto const &pointA = (*xyziCloud)[pointIndexA];
                Eigen::Vector3f pointAEigen(pointA.x, pointA.y, pointA.z);

                Eigen::Vector3f directionAB = pointBEigen - pointAEigen;
                directionAB /= directionAB.norm();

                for(size_t pointIndexIndexC = pointIndexIndexA + 1; pointIndexIndexC < nnFound; ++pointIndexIndexC)
                {
                    size_t const pointIndexC = nnIndices[pointIndexIndexC];
                    auto const &pointC = (*xyziCloud)[pointIndexC];
                    Eigen::Vector3f pointCEigen(pointC.x, pointC.y, pointC.z);

                    Eigen::Vector3f directionBC = pointCEigen - pointBEigen;
                    directionBC /= directionBC.norm();

                    float const angle = directionAB.dot(directionBC);

                    // calculate error
                    float const error = -0.5f * (angle - 1.0f);

                    if(error <= genTripletsMaxError)
                    {
                        // calculate center
                        Eigen::Vector3f center = (pointAEigen + pointBEigen + pointCEigen) / 3.0f;

                        // calculate direction
                        Eigen::Vector3f direction = pointCEigen - pointBEigen;
                        direction /= direction.norm();

                        Triplet newTriplet;

                        newTriplet.pointIndexA = pointIndexA;
                        newTriplet.pointIndexB = pointIndexB;
                        newTriplet.pointIndexC = pointIndexC;
                        newTriplet.center = center;
                        newTriplet.direction = direction;
                        newTriplet.error = error;

                        tripletCandidates.push_back(newTriplet);
                    }
                }
            }

            // order triplet candidates
            std::sort(tripletCandidates.begin(), tripletCandidates.end(), [](Triplet const &lhs, Triplet const &rhs) { return lhs.error < rhs.error; });

            // use the n best candidates
            for (size_t i = 0; i < std::min(genTripletsNBest, tripletCandidates.size()); ++i)
            {
                triplets.push_back(tripletCandidates[i]);
            }
        }
    }


}