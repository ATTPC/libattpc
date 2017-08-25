#include "catch.hpp"
#include "eigen_common.h"
#include "neighbors.h"

using attpc::cleaning::countNeighbors;

TEST_CASE("Can count neighbors", "[neighbors]") {
    const Eigen::Index numPts = 20;
    const double neighborRadius = 20;

    Eigen::ArrayX3d data {numPts, 3};
    for (Eigen::Index row = 0; row < data.rows(); ++row) {
        const double offset = row >= numPts / 2 ? 100 : 0;
        data.row(row).setConstant(row + offset);
    }

    Eigen::ArrayXi counts = countNeighbors(data.matrix(), neighborRadius);
    CAPTURE(counts);
    REQUIRE((counts == (numPts / 2 - 1)).all());
}

TEST_CASE("Points don't count themselves as neighbors", "[neighbors]") {
    const Eigen::Index numPts = 10;
    const double neighborRadius = 1;

    Eigen::ArrayX3d data {numPts, 3};
    for (Eigen::Index row = 0; row < data.rows(); ++row) {
        data.row(row).setConstant(row * 100);
    }

    Eigen::ArrayXi counts = countNeighbors(data.matrix(), neighborRadius);
    CAPTURE(counts);
    REQUIRE((counts == 0).all());
}
