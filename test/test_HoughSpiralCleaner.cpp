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

Eigen::ArrayX2d makeTestLineData(const double radius, const double theta,
                                 const double xmin, const double xmax, const int numPts) {
    Eigen::ArrayX2d data {numPts, 2};
    data.col(0) = Eigen::ArrayXd::LinSpaced(numPts, xmin, xmax);
    data.col(1) = (radius - data.col(0) * std::cos(theta)) / std::sin(theta);
    return data;
}

attpc::cleaning::HoughSpiralCleanerConfig makeConfig() {
    attpc::cleaning::HoughSpiralCleanerConfig config;
    config.linearHoughNumBins = 500;
    config.linearHoughMaxRadius = 20;
    config.circularHoughNumBins = 500;
    config.circularHoughMaxRadius = 20;
    config.numAngleBinsToReduce = 1;
    config.houghSpaceSliceSize = 5;

    return config;
}
}

TEST_CASE("HoughSpiralCleaner can find arc length of circle segment", "[houghcleaner]") {
    const Eigen::Vector2d center {4, 5};
    const int numPts = 100;
    const double minAngle = 0;
    const double maxAngle = M_PI / 2;
    const double radius = 3;

    Eigen::ArrayX2d testData = makeTestArcData(numPts, minAngle, maxAngle, radius, center);

    auto config = makeConfig();
    attpc::cleaning::HoughSpiralCleaner cleaner {config};

    const Eigen::ArrayXd arclenResult = cleaner.findArcLength(testData, center);

    SECTION("First point has arc length 0") {
        REQUIRE(arclenResult(0) == Approx(0.));
    }

    SECTION("Last point has arc length R*Pi/2") {
        REQUIRE(arclenResult(arclenResult.size() - 1) == Approx(radius * maxAngle));
    }
}

TEST_CASE("HoughSpiralCleaner can find max angle bin in Hough space", "[houghcleaner]") {
    SECTION("Works with a single point in Hough space") {
        auto config = makeConfig();
        config.numAngleBinsToReduce = 1;
        attpc::cleaning::HoughSpiralCleaner cleaner {config};

        const Eigen::Index maxRadiusBin = config.linearHoughNumBins / 2;
        const Eigen::Index maxAngleBin = config.linearHoughNumBins / 3;

        Eigen::ArrayXXd pointSpace = Eigen::ArrayXXd::Zero(config.linearHoughNumBins, config.linearHoughNumBins);
        pointSpace(maxAngleBin, maxRadiusBin) = 100;

        const Eigen::Index foundBin = cleaner.findMaxAngleBin(pointSpace);

        REQUIRE(foundBin == maxAngleBin);
    }

    SECTION("Works with a single line") {
        auto config = makeConfig();
        config.numAngleBinsToReduce = 1;
        attpc::cleaning::HoughSpiralCleaner cleaner {config};

        const double lineRad = 4;
        const double lineTheta = M_PI / 3;
        const double xMin = -10;
        const double xMax = 10;
        const int numPts = 100;

        const Eigen::ArrayX2d lineData = makeTestLineData(lineRad, lineTheta, xMin, xMax, numPts);

        attpc::cleaning::LinearHoughTransform trans {config.linearHoughNumBins, config.linearHoughMaxRadius};
        Eigen::ArrayXXd houghSpace = trans.findHoughSpace(lineData);

        const Eigen::Index foundBin = cleaner.findMaxAngleBin(houghSpace);
        const Eigen::Index expectedBin = trans.findBinFromAngle(lineTheta);
        REQUIRE(foundBin == expectedBin);
    }

    SECTION("Finds average with two parallel lines") {
        auto config = makeConfig();
        config.numAngleBinsToReduce = 5;
        attpc::cleaning::HoughSpiralCleaner cleaner {config};

        const double lineRad0 = 4;
        const double lineRad1 = 5;
        const double lineTheta0 = M_PI / 3;
        const double lineTheta1 = lineTheta0 * 1.01;
        const double xMin = -10;
        const double xMax = 10;
        const int numPtsEach = 100;

        Eigen::ArrayX2d lineData {numPtsEach * 2, 2};
        lineData.block(0, 0, numPtsEach, 2) = makeTestLineData(lineRad0, lineTheta0, xMin, xMax, numPtsEach);
        lineData.block(numPtsEach, 0, numPtsEach, 2) = makeTestLineData(lineRad1, lineTheta1, xMin, xMax, numPtsEach);

        attpc::cleaning::LinearHoughTransform trans {config.linearHoughNumBins, config.linearHoughMaxRadius};
        Eigen::ArrayXXd houghSpace = trans.findHoughSpace(lineData);

        const Eigen::Index foundBin = cleaner.findMaxAngleBin(houghSpace);
        const double expectedAngle = (lineTheta0 + lineTheta1) / 2;
        const Eigen::Index expectedBin = trans.findBinFromAngle(expectedAngle);
        REQUIRE(foundBin == expectedBin);
    }
}
