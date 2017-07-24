//
// Created by Joshua Bradt on 7/24/17.
//

#include "HTripletClustering.h"

namespace hc {
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