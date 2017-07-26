//
// Created by Joshua Bradt on 7/25/17.
//

#include "catch.hpp"
#include <Eigen/Core>
#include <limits>
#include "metrics.h"
#include "utilities.h"

namespace atc = attpc::cleaning;

static float euclideanMetric(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    return (a - b).norm();
}

class MetricTestData {
public:
    MetricTestData(const int numPts_)
    : numPts(numPts_)
    {
        for (int i = 0; i < numPts; ++i) {
            points.emplace_back(i, 2*i, 3*i);
        }

        distanceMatrix = attpc::cleaning::calculateDistanceMatrix<Eigen::Vector3f>(points, euclideanMetric);
    }

public:
    const int numPts;
    std::vector<Eigen::Vector3f> points;
    Eigen::ArrayXXf distanceMatrix;
};

TEST_CASE("Single-linkage metric works", "[metrics]") {
    MetricTestData testData {20};

    const atc::cluster clusterA = {1, 3, 5, 7};
    const atc::cluster clusterB = {0, 2, 4, 6, 8, 10, 12};

    float expected = std::numeric_limits<float>::infinity();
    for (auto i : clusterA) {
        for (auto j : clusterB) {
            if (testData.distanceMatrix(i, j) < expected) {
                expected = testData.distanceMatrix(i, j);
            }
        }
    }

    const float result = atc::singleLinkClusterMetric(clusterA, clusterB, testData.distanceMatrix);

    REQUIRE(result == expected);
}

TEST_CASE("Complete-linkage metric works", "[metrics]") {
    MetricTestData testData {20};

    const atc::cluster clusterA = {1, 3, 5, 7};
    const atc::cluster clusterB = {0, 2, 4, 6, 8, 10, 12};

    float expected = 0;
    for (auto i : clusterA) {
        for (auto j : clusterB) {
            if (testData.distanceMatrix(i, j) > expected) {
                expected = testData.distanceMatrix(i, j);
            }
        }
    }

    const float result = atc::completeLinkClusterMetric(clusterA, clusterB, testData.distanceMatrix);

    REQUIRE(result == expected);
}