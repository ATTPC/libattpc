#include "catch.hpp"
#include "LinearHoughTransform.h"
#include <Eigen/Core>
#include <vector>

TEST_CASE("Linear Hough can transform radii into bins", "[hough][bins]") {
    const int numBins = 500;
    const int maxRad = 2000;
    attpc::cleaning::LinearHoughTransform trans {numBins, maxRad};

    SECTION("Radius bin is 0 at lower radius bound") {
        REQUIRE(trans.findBinFromRadius(-maxRad) == 0);
    }

    SECTION("Radius bin is max at upper radius bound") {
        // Technically, this would be out-of-bounds, but the math is correct
        REQUIRE(trans.findBinFromRadius(maxRad) == numBins);
    }

    SECTION("Radius bin is in center at radius of 0") {
        REQUIRE(trans.findBinFromRadius(0) == numBins / 2);
    }

    SECTION("Radius transformation round trip is identity") {
        std::vector<Eigen::Index> bins {0, numBins / 2, numBins - 1};
        for (auto bin : bins) {
            CAPTURE(bin);
            auto rad = trans.findRadiusFromBin(bin);
            CAPTURE(rad);
            auto newBin = trans.findBinFromRadius(rad);
            CAPTURE(newBin);

            REQUIRE(newBin == bin);
        }
    }
}

TEST_CASE("Linear Hough can transform angles into bins", "[hough][bins][!mayfail]") {
    const int numBins = 500;
    const int maxRad = 2000;
    attpc::cleaning::LinearHoughTransform trans {numBins, maxRad};

    SECTION("Angle bin is 0 at 0 radians") {
        REQUIRE(trans.findBinFromAngle(0) == 0);
    }

    SECTION("Angle bin is max at pi radians") {
        // Technically, this would be out-of-bounds, but the math is correct
        REQUIRE(trans.findBinFromAngle(M_PI) == numBins);
    }

    SECTION("Angle transformation round trip is identity") {
        for (int bin = 0; bin < numBins; bin += 10) {
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
    const int maxRad = 2000;
    attpc::cleaning::LinearHoughTransform trans {numBins, maxRad};

    SECTION("Can find one horizontal line") {
        const Eigen::Index numPts = 10;
        Eigen::Array<double, Eigen::Dynamic, 2> data {};
        for (Eigen::Index row = 0; row < data.rows(); ++row) {
            data(row, 0) = row;
            data(row, 1) = 10;
        }

        CAPTURE(data);

        Eigen::ArrayXXd houghSpace = trans.findHoughSpace(data);

        SECTION("Hough space max equals the number of points") {
            double houghMax = houghSpace.maxCoeff();
            REQUIRE(houghMax == numPts);
        }
    }

}
