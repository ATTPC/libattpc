#include "catch.hpp"
#include "LinearHoughTransform.h"
#include <Eigen/Core>
#include <vector>

TEST_CASE("Linear Hough can transform radii into bins", "[hough][bins]") {
    const int numBins = 500;
    const int maxRad = 2000;
    attpc::cleaning::LinearHoughTransform trans {numBins, maxRad};

    SECTION("Radius bin is 0 at lower radius bound") {
        REQUIRE(trans.findBinFromRadius(trans.getMinRadiusValue()) == 0);
    }

    SECTION("Radius bin is max at upper radius bound") {
        // Technically, this would be out-of-bounds, but the math is correct
        REQUIRE(trans.findBinFromRadius(trans.getMaxRadiusValue()) == numBins);
    }

    SECTION("Radius bin is in center at radius of 0") {
        REQUIRE(trans.findBinFromRadius(0) == numBins / 2);
    }

    SECTION("Radius transformation round trip is identity") {
        for (Eigen::Index bin = 0; bin < numBins; ++bin) {
            CAPTURE(bin);
            auto rad = trans.findRadiusFromBin(bin);
            CAPTURE(rad);
            auto newBin = trans.findBinFromRadius(rad);
            CAPTURE(newBin);

            REQUIRE(newBin == bin);
        }
    }
}

TEST_CASE("Linear Hough can transform angles into bins", "[hough][bins]") {
    const int numBins = 500;
    const int maxRad = 2000;
    attpc::cleaning::LinearHoughTransform trans {numBins, maxRad};

    SECTION("Angle bin is 0 at minimum angle") {
        REQUIRE(trans.findBinFromAngle(trans.getMinAngleValue()) == 0);
    }

    SECTION("Angle bin is max at maximum angle") {
        // Technically, this would be out-of-bounds, but the math is correct
        REQUIRE(trans.findBinFromAngle(trans.getMaxAngleValue()) - numBins <= 1);  // Some floating-point imprecision
    }

    SECTION("Angle transformation round trip is identity") {
        for (Eigen::Index bin = 0; bin < numBins; ++bin) {
            CAPTURE(bin);
            double angle = trans.findAngleFromBin(bin);
            CAPTURE(angle);
            Eigen::Index newBin = trans.findBinFromAngle(angle);
            CAPTURE(newBin);

            REQUIRE(std::abs(newBin - bin) <= 1);  // This may be a bit off due to floating point errors
        }
    }
}

TEST_CASE("Linear Hough transform finds lines", "[hough]") {
    const int numBins = 500;
    const int maxRad = 20;
    attpc::cleaning::LinearHoughTransform trans {numBins, maxRad};

    SECTION("Can find one horizontal line") {
        const Eigen::Index numPts = 10;
        const double dataY = 11;
        Eigen::Array<double, numPts, 2> data {};
        for (Eigen::Index row = 0; row < data.rows(); ++row) {
            data(row, 0) = row;
            data(row, 1) = dataY;
        }

        CAPTURE(data);

        Eigen::ArrayXXd houghSpace = trans.findHoughSpace(data);

        SECTION("Hough space max equals the number of points") {
            double houghMax = houghSpace.maxCoeff();
            REQUIRE(houghMax == numPts);
        }

        SECTION("Hough space max is in the expected bins") {
            Eigen::Index radMaxBin = 0;
            Eigen::Index thetaMaxBin = 0;
            houghSpace.maxCoeff(&thetaMaxBin, &radMaxBin);

            CAPTURE(thetaMaxBin);
            CAPTURE(radMaxBin);

            double thetaMax = trans.findAngleFromBin(thetaMaxBin);
            double radMax = trans.findRadiusFromBin(radMaxBin);

            CAPTURE(thetaMax);
            CAPTURE(radMax);

            CHECK(radMax == Approx(dataY).margin(trans.getRadiusBinSize()));
            CHECK(thetaMax == Approx(trans.getMaxAngleValue() / 2).margin(trans.getAngleBinSize()));
        }
    }

}
