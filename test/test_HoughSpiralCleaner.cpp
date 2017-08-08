#include "catch.hpp"
#include "HoughSpiralCleaner.h"

namespace {
Eigen::ArrayX2d makeTestArcData(const int numPts, const double minAngle, const double maxAngle,
                                const double radius, const Eigen::Vector2d& center) {
    const Eigen::ArrayXd angles = Eigen::ArrayXd::LinSpaced(numPts, minAngle, maxAngle);
    Eigen::ArrayX2d data {numPts, 2};
    data.col(0) = Eigen::cos(angles) * radius + center(0);
    data.col(1) = Eigen::sin(angles) * radius + center(1);

    return data;
}
}

TEST_CASE("HoughSpiralCleaner can find arc length of circle segment", "[houghcleaner]") {
    const Eigen::Vector2d center {4, 5};
    const int numPts = 100;
    const double minAngle = 0;
    const double maxAngle = M_PI / 2;
    const double radius = 3;

    Eigen::ArrayX2d testData = makeTestArcData(numPts, minAngle, maxAngle, radius, center);

    attpc::cleaning::HoughSpiralCleaner cleaner {};

    const Eigen::ArrayXd arclenResult = cleaner.findArcLength(testData, center);

    SECTION("First point has arc length 0") {
        REQUIRE(arclenResult(0) == Approx(0.));
    }

    SECTION("Last point has arc length R*Pi/2") {
        REQUIRE(arclenResult(arclenResult.size() - 1) == Approx(radius * maxAngle));
    }
}
