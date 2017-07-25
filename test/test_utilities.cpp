//
// Created by Joshua Bradt on 7/25/17.
//

#include "catch.hpp"
#include <Eigen/Core>
#include "utilities.h"

static float euclideanMetric(const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
    return (a - b).norm();
}


TEST_CASE("Can calculate distance matrix", "[utilities]") {
    const int numPoints = 100;

    std::vector<Eigen::Vector3f> data;
    for (int i = 0; i < numPoints; ++i) {
        data.emplace_back(i, i, i);
    }

    Eigen::MatrixXf distMat = attpc::clustering::calculateDistanceMatrix<Eigen::Vector3f>(data, euclideanMetric);

    SECTION("Distance matrix is symmetric") {
        REQUIRE(distMat == distMat.transpose());
    }

    SECTION("Distances are calculated correctly") {
        for (int i = 0; i < numPoints; ++i) {
            for (int j = 0; j < numPoints; ++j) {
                float expected = euclideanMetric(data.at(i), data.at(j));
                REQUIRE(distMat(i, j) == expected);
            }
        }
    }
}