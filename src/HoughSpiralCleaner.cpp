//
// Created by Joshua Bradt on 7/28/17.
//

#include "HoughSpiralCleaner.h"

namespace attpc {
namespace cleaning {

HoughSpiralCleaner::HoughSpiralCleaner()
: numAngleBinsToReduce(5)
, houghSpaceSliceSize(5)
{}

Eigen::ArrayXd HoughSpiralCleaner::findArcLength(const Eigen::ArrayXXd& xy, const Eigen::Vector2d center) const {
    const Eigen::ArrayXd xOffset = xy.col(0) - center(0);
    const Eigen::ArrayXd yOffset = xy.col(1) - center(1);

    Eigen::ArrayXd rads = Eigen::sqrt(xOffset.square() + yOffset.square());
    Eigen::ArrayXd thetas = Eigen::atan(yOffset / xOffset);

    return rads * thetas;
}

Eigen::Index HoughSpiralCleaner::findMaxAngleBin(const Eigen::ArrayXXd& houghSpace) const {
    // Create a flattened 1D view of the Hough space
    const Eigen::Map<const Eigen::ArrayXd> flatHoughSpace (houghSpace.data(), houghSpace.size());

    // Find the ordering of indices that would sort the array
    std::vector<Eigen::Index> flatIndices (flatHoughSpace.size());
    std::iota(flatIndices.begin(), flatIndices.end(), 0);
    std::sort(flatIndices.begin(), flatIndices.end(), [&flatHoughSpace](auto idxA, auto idxB) {
        return flatHoughSpace[idxA] < flatHoughSpace[idxB];
    });

    // Find the mean bin from the last few elements of the sorted list
    double binTotal = std::accumulate(flatIndices.end() - numAngleBinsToReduce, flatIndices.end(), 0);
    Eigen::Index meanBin = static_cast<Eigen::Index>(std::floor(binTotal / numAngleBinsToReduce));

    return meanBin;
}

Eigen::ArrayXd HoughSpiralCleaner::findMaxAngleSlice(const Eigen::ArrayXXd& houghSpace,
                                                     const Eigen::Index maxAngleBin) const {
    return houghSpace.block(maxAngleBin - houghSpaceSliceSize, 0, 2 * houghSpaceSliceSize, houghSpace.rows())
                     .colwise().sum();
}

}
}
