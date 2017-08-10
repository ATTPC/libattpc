#include "catch.hpp"
#include "HoughSpace.h"
#include <Eigen/Core>

TEST_CASE("HoughSpace can transform radii into bins", "[hough][bins]") {
    const Eigen::Index numBins = 500;
    const double maxRad = 2000;
    attpc::cleaning::HoughSpace hspace {numBins, maxRad};

    SECTION("Radius bin is 0 at lower radius bound") {
        REQUIRE(hspace.findBinFromRadius(hspace.getMinRadiusValue()) == 0);
    }

    SECTION("Radius bin is max at upper radius bound") {
        // Technically, this would be out-of-bounds, but the math is correct
        REQUIRE(hspace.findBinFromRadius(hspace.getMaxRadiusValue()) == numBins);
    }

    SECTION("Radius bin is in center at radius of 0") {
        REQUIRE(hspace.findBinFromRadius(0) == numBins / 2);
    }

    SECTION("Radius transformation round trip is identity") {
        for (Eigen::Index bin = 0; bin < numBins; ++bin) {
            CAPTURE(bin);
            auto rad = hspace.findRadiusFromBin(bin);
            CAPTURE(rad);
            auto newBin = hspace.findBinFromRadius(rad);
            CAPTURE(newBin);

            REQUIRE(newBin == bin);
        }
    }
}

TEST_CASE("HoughSpace can transform angles into bins", "[hough][bins]") {
    const Eigen::Index numBins = 500;
    const double maxRad = 2000;
    attpc::cleaning::HoughSpace hspace {numBins, maxRad};

    SECTION("Angle bin is 0 at minimum angle") {
        REQUIRE(hspace.findBinFromAngle(hspace.getMinAngleValue()) == 0);
    }

    SECTION("Angle bin is max at maximum angle") {
        // Technically, this would be out-of-bounds, but the math is correct
        REQUIRE(hspace.findBinFromAngle(hspace.getMaxAngleValue()) - numBins <= 1);  // Some floating-point imprecision
    }

    SECTION("Angle transformation round trip is identity") {
        for (Eigen::Index bin = 0; bin < numBins; ++bin) {
            CAPTURE(bin);
            double angle = hspace.findAngleFromBin(bin);
            CAPTURE(angle);
            Eigen::Index newBin = hspace.findBinFromAngle(angle);
            CAPTURE(newBin);

            REQUIRE(std::abs(newBin - bin) <= 1);  // This may be a bit off due to floating point errors
        }
    }
}
