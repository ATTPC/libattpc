#include "catch.hpp"
#include "attpc/cleaning/LinearHoughTransform.h"
#include "attpc/cleaning/eigen_common.h"
#include <vector>

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

        attpc::cleaning::HoughSpace houghSpace = trans.findHoughSpace(data.col(0), data.col(1));

        SECTION("Hough space max equals the number of points") {
            auto houghMax = houghSpace.findMaximum();
            REQUIRE(houghMax == numPts);
        }

        SECTION("Hough space max is in the expected bins") {
            Eigen::Index radMaxBin = 0;
            Eigen::Index thetaMaxBin = 0;
            houghSpace.findMaximum(thetaMaxBin, radMaxBin);

            CAPTURE(thetaMaxBin);
            CAPTURE(radMaxBin);

            double thetaMax = houghSpace.findAngleFromBin(thetaMaxBin);
            double radMax = houghSpace.findRadiusFromBin(radMaxBin);

            CAPTURE(thetaMax);
            CAPTURE(radMax);

            CHECK(radMax == Approx(dataY).margin(houghSpace.getRadiusBinSize()));
            CHECK(thetaMax == Approx(houghSpace.getMaxAngleValue() / 2).margin(houghSpace.getAngleBinSize()));
        }
    }

}
