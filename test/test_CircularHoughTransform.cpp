#include "catch.hpp"
#include "CircularHoughTransform.h"
#include "eigen_common.h"
#include <cmath>

using attpc::cleaning::CircularHoughTransform;

TEST_CASE("Circular Hough can find center of a circle", "[hough]") {
    const Eigen::Vector2d center {1, 2};
    const double radius = 4;
    const Eigen::Index numPts = 100;

    const Eigen::ArrayXd angles = Eigen::ArrayXd::LinSpaced(numPts, 0, 2*M_PI);
    Eigen::ArrayX2d data (angles.rows(), 2);
    data.col(0) = radius * Eigen::cos(angles) + center(0);
    data.col(1) = radius * Eigen::sin(angles) + center(1);

    const int numBins = 500;
    const int maxRadius = 20;

    CircularHoughTransform trans {numBins, maxRadius};

    Eigen::Vector2d foundCenter = trans.findCenter(data.col(0), data.col(1));

    CHECK(foundCenter(0) == Approx(center(0)).margin(0.1));
    CHECK(foundCenter(1) == Approx(center(1)).margin(0.1));
}

TEST_CASE("Circular Hough throws an exception with too few points") {
    const int numPts = 5;
    const int numBins = 500;
    const int maxRadius = 20;
    const int rowOffset = 10;

    const Eigen::ArrayX2d data = Eigen::ArrayX2d::Zero(numPts, 2);  // It doesn't matter what the data is, in this case

    CircularHoughTransform trans {numBins, maxRadius, rowOffset};

    CHECK_THROWS_AS(trans.findHoughSpace(data.col(0), data.col(1)), const CircularHoughTransform::TooFewPointsException&);
    CHECK_THROWS_AS(trans.findCenter(data.col(0), data.col(1)), const CircularHoughTransform::TooFewPointsException&);
}
