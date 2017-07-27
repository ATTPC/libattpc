//
// Created by Joshua Bradt on 7/24/17.
//

#include "HTripletClustering.h"

namespace attpc {
namespace cleaning {

HTripletClustering::HTripletClustering()
        : clusterMetric(singleLinkClusterMetric)
        , tripletMetric(spiralTripletMetric) {}

HTripletClustering::HTripletClustering(const HTripletClusteringConfig& config) {
    setParamsFromConfig(config);
}

void HTripletClustering::setParamsFromConfig(const HTripletClusteringConfig& config) {
    genTripletsNnCandidates = config.genTripletsNnCandidates;
    genTripletsNBest = config.genTripletsNBest;
    cleanupMinTriplets = config.cleanupMinTriplets;
    smoothRadius = config.smoothRadius;
    genTripletsMaxError = config.genTripletsMaxError;
    bestClusterDistanceDelta = config.bestClusterDistanceDelta;
    smoothUsingMedian = config.smoothUsingMedian;
    clusterMetric = config.clusterMetric;
    tripletMetric = config.tripletMetric;
}

HTripletClustering::cloud_type HTripletClustering::smoothCloud(cloud_type::ConstPtr cloud) const {
    cloud_type smoothCloud;

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

    kdtree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i) {
        pcl::PointXYZI newPoint;

        pcl::CentroidPoint<pcl::PointXYZI> centroid;
        pcl::PointXYZI centroidPoint;
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        int found = kdtree.radiusSearch(*cloud, (int) i, smoothRadius, pointIdxNKNSearch, pointNKNSquaredDistance);

        for (int j = 0; j < found; ++j) {
            centroid.add((*cloud)[pointIdxNKNSearch[j]]);
        }

        centroid.get(centroidPoint);

        if (smoothUsingMedian) {
            std::vector<float> xList;
            std::vector<float> yList;
            std::vector<float> zList;

            for (int j = 0; j < found; ++j) {
                pcl::PointXYZI const& point = (*cloud)[pointIdxNKNSearch[j]];

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

        smoothCloud.push_back(newPoint);
    }

    return smoothCloud;
}

std::vector<Triplet> HTripletClustering::generateTriplets(cloud_type::ConstPtr cloud) const {
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<Triplet> triplets;

    for (size_t pointIndexB = 0; pointIndexB < cloud->size(); ++pointIndexB) {
        auto const& pointB = (*cloud)[pointIndexB];
        Eigen::Vector3f pointBEigen(pointB.x, pointB.y, pointB.z);

        std::vector<Triplet> tripletCandidates;

        std::vector<int> nnIndices;
        nnIndices.reserve(genTripletsNnCandidates);
        std::vector<float> nnbSquaredDistances;
        nnbSquaredDistances.reserve(genTripletsNnCandidates);
        int const nnFound = kdtree.nearestKSearch(*cloud, (int) pointIndexB, (int) genTripletsNnCandidates, nnIndices,
                                                  nnbSquaredDistances);

        for (size_t pointIndexIndexA = 0; pointIndexIndexA < nnFound; ++pointIndexIndexA) {
            size_t const pointIndexA = nnIndices[pointIndexIndexA];
            auto const& pointA = (*cloud)[pointIndexA];
            Eigen::Vector3f pointAEigen(pointA.x, pointA.y, pointA.z);

            Eigen::Vector3f directionAB = pointBEigen - pointAEigen;
            directionAB /= directionAB.norm();

            for (size_t pointIndexIndexC = pointIndexIndexA + 1; pointIndexIndexC < nnFound; ++pointIndexIndexC) {
                size_t const pointIndexC = nnIndices[pointIndexIndexC];
                auto const& pointC = (*cloud)[pointIndexC];
                Eigen::Vector3f pointCEigen(pointC.x, pointC.y, pointC.z);

                Eigen::Vector3f directionBC = pointCEigen - pointBEigen;
                directionBC /= directionBC.norm();

                float const angle = directionAB.dot(directionBC);

                // calculate error
                float const error = -0.5f * (angle - 1.0f);

                if (error <= genTripletsMaxError) {
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
        std::sort(tripletCandidates.begin(), tripletCandidates.end(),
                  [](Triplet const& lhs, Triplet const& rhs) { return lhs.error < rhs.error; });

        // use the n best candidates
        for (size_t i = 0; i < std::min(genTripletsNBest, tripletCandidates.size()); ++i) {
            triplets.push_back(tripletCandidates[i]);
        }
    }

    return triplets;
}

cluster_history
HTripletClustering::calculateHc(cloud_type::ConstPtr cloud, const std::vector<Triplet>& triplets) const {
    cluster_history result;

    result.triplets = triplets;

    // calculate distance-Matrix
    Eigen::MatrixXf distanceMatrix = calculateDistanceMatrix(result.triplets, tripletMetric);

    cluster_group currentGeneration;

    // init first generation
    for (size_t i = 0; i < result.triplets.size(); ++i) {
        cluster newCluster;
        newCluster.push_back(i);
        currentGeneration.clusters.push_back(newCluster);
    }

    // merge until only one cluster left
    while (currentGeneration.clusters.size() > 1) {
        float bestClusterDistance = std::numeric_limits<float>::infinity();
        std::pair<size_t, size_t> bestClusterPair;

        // find best cluster-pair
        for (size_t i = 0; i < currentGeneration.clusters.size(); ++i) {
            for (size_t j = i + 1; j < currentGeneration.clusters.size(); ++j) {
                float const clusterDistance = clusterMetric(currentGeneration.clusters[i],
                                                            currentGeneration.clusters[j], distanceMatrix);

                if (clusterDistance < bestClusterDistance) {
                    bestClusterDistance = clusterDistance;
                    bestClusterPair = std::pair<size_t, size_t>(i, j);
                }
            }
        }

        // merge cluster-pair
        cluster merged(currentGeneration.clusters[bestClusterPair.first]);
        merged.insert(merged.cend(), currentGeneration.clusters[bestClusterPair.second].cbegin(),
                      currentGeneration.clusters[bestClusterPair.second].cend());

        // set best cluster distance for current generation and add to results
        currentGeneration.bestClusterDistance = bestClusterDistance;
        result.history.push_back(currentGeneration);

        // copy data to next generation
        cluster_group nextGeneration;
        nextGeneration.clusters.reserve(currentGeneration.clusters.size() - 1);

        for (size_t i = 0; i < currentGeneration.clusters.size(); ++i) {
            if (i != bestClusterPair.first && i != bestClusterPair.second)
                nextGeneration.clusters.push_back(currentGeneration.clusters[i]);
        }

        nextGeneration.clusters.push_back(merged);

        // overwrite current generation
        currentGeneration = nextGeneration;
    }

    // set last cluster distance and add to results
    currentGeneration.bestClusterDistance = std::numeric_limits<float>::infinity();
    result.history.push_back(currentGeneration);

    return result;
}

cluster_group HTripletClustering::findBestClusterGroup(const cluster_history& history) const {
    float lastBestClusterDistance = history.history[0].bestClusterDistance;

    for (const cluster_group& clusterGroup : history.history) {
        const float bestClusterDistanceChange = clusterGroup.bestClusterDistance - lastBestClusterDistance;
        lastBestClusterDistance = clusterGroup.bestClusterDistance;

        if (bestClusterDistanceChange > bestClusterDistanceDelta) {
            return clusterGroup;
        }
    }

    return history.history[history.history.size() - 1];
}

cluster_group HTripletClustering::cleanupClusterGroup(cluster_group const& clusterGroup) const {
    cluster_group cleanedGroup;
    cleanedGroup.bestClusterDistance = clusterGroup.bestClusterDistance;
    cleanedGroup.clusters.resize(clusterGroup.clusters.size());

    auto newEnd = std::copy_if(clusterGroup.clusters.cbegin(), clusterGroup.clusters.cend(),
                               cleanedGroup.clusters.begin(), [&](cluster const& cluster) {
                return cluster.size() >= cleanupMinTriplets;
            });
    cleanedGroup.clusters.resize(std::distance(cleanedGroup.clusters.begin(), newEnd));

    return cleanedGroup;
}

Cluster HTripletClustering::makeCluster(const std::vector<Triplet>& triplets, const cluster_group& clusterGroup,
                                        size_t pointIndexCount) const {
    std::vector<pcl::PointIndicesPtr> result;

    for (auto const& currentCluster : clusterGroup.clusters) {
        pcl::PointIndicesPtr pointIndices(new pcl::PointIndices());

        // add point indices
        for (auto const& currentTripletIndex : currentCluster) {
            Triplet const& currentTriplet = triplets[currentTripletIndex];

            pointIndices->indices.push_back((int) currentTriplet.pointIndexA);
            pointIndices->indices.push_back((int) currentTriplet.pointIndexB);
            pointIndices->indices.push_back((int) currentTriplet.pointIndexC);
        }

        // sort point-indices and remove duplikates
        std::sort(pointIndices->indices.begin(), pointIndices->indices.end());
        auto newEnd = std::unique(pointIndices->indices.begin(), pointIndices->indices.end());
        pointIndices->indices.resize(std::distance(pointIndices->indices.begin(), newEnd));

        result.push_back(pointIndices);
    }

    return Cluster(result, pointIndexCount);
}

}
}