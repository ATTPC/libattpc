#include "catch.hpp"
#include "HoughSpace.h"
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

Eigen::ArrayX3d makeTest3dArcData(const int numPts, const double minAngle, const double maxAngle,
                                const double radius, const Eigen::Vector2d& center) {
    const Eigen::ArrayXd angles = Eigen::ArrayXd::LinSpaced(numPts, minAngle, maxAngle);
    Eigen::ArrayX3d data {numPts, 3};
    data.col(0) = Eigen::cos(angles) * radius + center(0);
    data.col(1) = Eigen::sin(angles) * radius + center(1);
    data.col(2) = Eigen::ArrayXd::LinSpaced(numPts, 10, 2);

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
    config.houghSpaceSliceThreshold = 0;
    config.peakWidth = 5;
    config.minPointsPerLine = 5;
    config.neighborRadius = 100;

    return config;
}
}

TEST_CASE("HoughSpiralCleaner can find arc length of circle segment", "[houghcleaner]") {
    const Eigen::Vector2d center {4, 5};
    const int numPts = 100;
    const double minAngle = 0;
    const double maxAngle = M_PI;
    const double radius = 3;

    Eigen::ArrayX2d testData = makeTestArcData(numPts, minAngle, maxAngle, radius, center);

    auto config = makeConfig();
    attpc::cleaning::HoughSpiralCleaner cleaner {config};

    const Eigen::ArrayXd arclenResult = cleaner.findArcLength(testData, center);

    SECTION("First point has arc length 0") {
        REQUIRE(arclenResult(0) == Approx(0.));
    }

    SECTION("Last point has arc length R*Pi") {
        REQUIRE(std::abs(arclenResult(arclenResult.size() - 1)) == Approx(radius * maxAngle));
    }

    SECTION("Branch cut is at pi / 2") {
        CHECK((arclenResult >= 0).all());

        Eigen::ArrayX2d testDataShifted = makeTestArcData(numPts, minAngle + M_PI/2, maxAngle + M_PI/2, radius, center);
        const Eigen::ArrayXd arclenResultShifted = cleaner.findArcLength(testDataShifted, center);
        CHECK(!(arclenResultShifted >= 0).all());
    }
}

TEST_CASE("HoughSpiralCleaner can perform a Hough transform", "[houghcleaner][hough]") {
    const Eigen::Index numPts = 10;
    const double yValue = 10;
    Eigen::ArrayX2d data {numPts, 2};
    data.col(0).setLinSpaced(-10, 10);
    data.col(1).setConstant(yValue);

    auto config = makeConfig();
    attpc::cleaning::HoughSpiralCleaner cleaner {config};

    attpc::cleaning::HoughSpace hspace = cleaner.findHoughSpace(data.col(0), data.col(1));

    Eigen::Index maxAngleBin, maxRadBin;
    const auto maxValue = hspace.findMaximum(maxAngleBin, maxRadBin);

    SECTION("Max in Hough space has the right amplitude") {
        REQUIRE(maxValue == numPts);
    }

    SECTION("Max in Hough space is in the right location") {
        CHECK(maxAngleBin == Approx(hspace.findBinFromAngle(M_PI / 2)).margin(1.1));
        CHECK(maxRadBin == Approx(hspace.findBinFromRadius(yValue)).margin(1.1));
    }
}

TEST_CASE("HoughSpiralCleaner can find max angle bin in Hough space", "[houghcleaner]") {
    SECTION("Works with a single point in Hough space") {
        auto config = makeConfig();
        config.numAngleBinsToReduce = 1;
        attpc::cleaning::HoughSpiralCleaner cleaner {config};

        const Eigen::Index maxRadiusBin = config.linearHoughNumBins / 2;
        const Eigen::Index maxAngleBin = config.linearHoughNumBins / 3;

        attpc::cleaning::HoughSpace pointSpace {config.linearHoughNumBins, config.linearHoughMaxRadius};
        pointSpace.getValueAtBin(maxAngleBin, maxRadiusBin) = 100;

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
        attpc::cleaning::HoughSpace houghSpace = trans.findHoughSpace(lineData.col(0), lineData.col(1));

        const Eigen::Index foundBin = cleaner.findMaxAngleBin(houghSpace);
        const Eigen::Index expectedBin = houghSpace.findBinFromAngle(lineTheta);
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
        attpc::cleaning::HoughSpace houghSpace = trans.findHoughSpace(lineData.col(0), lineData.col(1));

        const Eigen::Index foundBin = cleaner.findMaxAngleBin(houghSpace);
        const double expectedAngle = (lineTheta0 + lineTheta1) / 2;
        const Eigen::Index expectedBin = houghSpace.findBinFromAngle(expectedAngle);
        REQUIRE(foundBin == expectedBin);
    }
}

TEST_CASE("HoughSpiralCleaner can extract angular slice", "[houghcleaner]") {
    using attpc::cleaning::HoughSpiralCleaner;
    using attpc::cleaning::HoughSpace;

    auto config = makeConfig();
    config.houghSpaceSliceSize = 5;
    HoughSpiralCleaner cleaner {config};

    HoughSpace testData {config.linearHoughNumBins, config.linearHoughMaxRadius};

    SECTION("Slice is correct when only one angular bin is nonzero") {
        const Eigen::Index targetAngleBin = static_cast<Eigen::Index>(config.linearHoughNumBins * 0.75);
        testData.getAngularSlice(targetAngleBin) = 100;

        HoughSpiralCleaner::AngleSliceArrayType slice = cleaner.findMaxAngleSlice(testData, targetAngleBin);

        SECTION("Dimensions of extracted slice are correct") {
            REQUIRE(slice.size() == config.linearHoughNumBins);
        }

        SECTION("Values in extracted slice are correct") {
            Eigen::Array<HoughSpace::ScalarType, Eigen::Dynamic, Eigen::Dynamic> expected =
                testData.getAngularSlice(targetAngleBin);
            expected.resize(testData.getNumBins(), 1);
            REQUIRE((slice == expected).all());
        }
    }

    SECTION("Slice excludes values outside the slice region") {
        const Eigen::Index targetAngleBin = static_cast<Eigen::Index>(config.linearHoughNumBins * 0.75);
        const Eigen::Index otherAngleBin = static_cast<Eigen::Index>(config.linearHoughNumBins * 0.25);
        const attpc::cleaning::HoughSpace::ScalarType targetVal = 100;
        const attpc::cleaning::HoughSpace::ScalarType otherVal = 50;

        testData.getAngularSlice(targetAngleBin) = targetVal;
        testData.getAngularSlice(otherAngleBin) = otherVal;

        HoughSpiralCleaner::AngleSliceArrayType slice = cleaner.findMaxAngleSlice(testData, targetAngleBin);

        SECTION("Dimensions of extracted slice are correct") {
            REQUIRE(slice.size() == config.linearHoughNumBins);
        }

        SECTION("Values in extracted slice are correct") {
            Eigen::Array<HoughSpace::ScalarType, Eigen::Dynamic, Eigen::Dynamic> expected =
                testData.getAngularSlice(targetAngleBin);
            expected.resize(testData.getNumBins(), 1);
            REQUIRE((slice == expected).all());
        }
    }

    SECTION("Slice sums across slice region when multiple columns are nonzero") {
        const Eigen::Index targetAngleBin = static_cast<Eigen::Index>(config.linearHoughNumBins * 0.75);
        const attpc::cleaning::HoughSpace::ScalarType targetVal = 100;

        testData.getAngularSlice(targetAngleBin - 1) = targetVal;
        testData.getAngularSlice(targetAngleBin) = targetVal;
        testData.getAngularSlice(targetAngleBin + 1) = targetVal;

        HoughSpiralCleaner::AngleSliceArrayType slice = cleaner.findMaxAngleSlice(testData, targetAngleBin);

        SECTION("Dimensions of extracted slice are correct") {
            REQUIRE(slice.size() == config.linearHoughNumBins);
        }

        SECTION("Values in extracted slice are correct") {
            Eigen::Array<HoughSpace::ScalarType, Eigen::Dynamic, Eigen::Dynamic> expected =
                testData.getAngularSlice(targetAngleBin) * 3;
            expected.resize(testData.getNumBins(), 1);
            REQUIRE((slice == expected).all());
        }
    }

}

TEST_CASE("HoughSpiralCleaner can find peaks in Hough space slice", "[houghcleaner]") {
    using attpc::cleaning::HoughSpiralCleaner;

    auto config = makeConfig();
    HoughSpiralCleaner cleaner {config};

    HoughSpiralCleaner::AngleSliceArrayType testData = decltype(testData)::Zero(config.linearHoughNumBins);

    SECTION("Can find one asymmetric peak") {
        testData.segment(10, 5) << 5, 50, 20, 10, 5;

        const std::vector<double> foundPeaks = cleaner.findPeakRadiusBins(testData);

        SECTION("Found only one peak") {
            REQUIRE(foundPeaks.size() == 1);
        }

        SECTION("Peak was at correct location") {
            const Eigen::VectorXd peakValues = testData.segment(11 - config.peakWidth, 2*config.peakWidth + 1).cast<double>();
            const Eigen::VectorXd peakIndices =
                Eigen::VectorXd::LinSpaced(2 * config.peakWidth + 1, 11 - config.peakWidth, 11 + config.peakWidth);
            const double expectedValue = peakValues.dot(peakIndices) / peakValues.sum();
            REQUIRE(foundPeaks.at(0) == Approx(expectedValue));
        }
    }

    SECTION("Centers of gravity of several symmetric peaks are at the maxima") {
        const std::vector<Eigen::Index> peakLocs = {10, 30, 50, 70, 90};
        for (const auto loc : peakLocs) {
            testData(loc) = 100;
            testData(loc - 1) = 50;
            testData(loc + 1) = 50;
        }

        const std::vector<double> foundPeaks = cleaner.findPeakRadiusBins(testData);

        REQUIRE(foundPeaks.size() == peakLocs.size());

        CAPTURE(peakLocs);
        CAPTURE(foundPeaks);

        for (size_t i = 0; i < peakLocs.size(); ++i) {
            REQUIRE(peakLocs.at(i) == Approx(foundPeaks.at(i)));
        }
    }

    SECTION("Behavior at edges of array is correct") {
        SECTION("Peak at upper bound doesn't go out of bounds") {
            testData(testData.rows() - 1) = 100;
            REQUIRE_NOTHROW(cleaner.findPeakRadiusBins(testData));
        }

        SECTION("Peak at lower bound doesn't go out of bounds") {
            testData(0) = 100;
            REQUIRE_NOTHROW(cleaner.findPeakRadiusBins(testData));
        }
    }
}

TEST_CASE("HoughSpiralCleaner can classify points (simple example)", "[houghcleaner]") {
    // Test the simple example of a set of horizontal lines in arclen-z space with points near them.
    // This is, of course, not a realistic data set.
    using attpc::cleaning::HoughSpiralCleaner;
    using attpc::cleaning::HoughSpiralCleanerResult;

    Eigen::Array3d radii;
    radii << 1, 2, 3;

    const int pointsPerLine = 100;
    const double randomDistScale = 0.1;
    const double maxAngle = M_PI / 2;

    Eigen::ArrayX2d testData {pointsPerLine * radii.size(), 2};
    for (Eigen::Index radIdx = 0; radIdx < radii.rows(); ++radIdx) {
        testData.block<pointsPerLine, 1>(radIdx * pointsPerLine, 0).setLinSpaced(0, 1000);
        testData.block<pointsPerLine, 1>(radIdx * pointsPerLine, 1) =
            Eigen::ArrayXd::Constant(pointsPerLine, radii(radIdx)) +
            Eigen::ArrayXd::Random(pointsPerLine) * randomDistScale;
    }

    auto config = makeConfig();
    HoughSpiralCleaner cleaner {config};

    HoughSpiralCleanerResult result = cleaner.classifyPoints(testData.col(0), testData.col(1), maxAngle, radii);

    SECTION("Labels are correct") {
        // Note: This depends on the order in which the function traverses the list of radii
        for (Eigen::Index radIdx = 0; radIdx < radii.rows(); ++radIdx) {
            CHECK((result.labels.segment(radIdx * pointsPerLine, pointsPerLine) == radIdx).all());
        }
    }

    SECTION("Distances are correct") {
        for (Eigen::Index radIdx = 0; radIdx < radii.rows(); ++radIdx) {
            Eigen::ArrayXd expected =
                Eigen::abs(testData.block<pointsPerLine, 1>(radIdx * pointsPerLine, 1) - radii(radIdx));
            const Eigen::ArrayXd actual = result.distancesToNearestLine.segment(radIdx * pointsPerLine, pointsPerLine);
            CHECK((Eigen::abs(actual - expected) < 1e-4).all());
        }
    }

}

TEST_CASE("HoughSpiralCleaner processEvent works", "[houghcleaner]") {
    using attpc::cleaning::HoughSpiralCleaner;
    using attpc::cleaning::HoughSpiralCleanerResult;

    const double arcRadius = 4;
    const Eigen::Vector2d arcCenter = {2, 4};
    const Eigen::Index numPts = 100;
    const double minAngle = 0;
    const double maxAngle = 3 * M_PI / 4;

    const Eigen::ArrayXXd testData = makeTest3dArcData(numPts, minAngle, maxAngle, arcRadius, arcCenter);

    auto config = makeConfig();
    HoughSpiralCleaner cleaner {config};

    HoughSpiralCleanerResult result = cleaner.processEvent(testData);

    SECTION("Point labels are correct") {
        CAPTURE(result.labels);
        REQUIRE((result.labels == 0).all());
    }

    SECTION("Distances are correct") {
        CAPTURE(result.distancesToNearestLine);
        REQUIRE((Eigen::abs(result.distancesToNearestLine) < 0.2).all());
    }

    SECTION("Found center is correct") {
        CHECK(result.center(0) == Approx(arcCenter(0)).margin(0.1));
        CHECK(result.center(1) == Approx(arcCenter(1)).margin(0.1));
    }
}
