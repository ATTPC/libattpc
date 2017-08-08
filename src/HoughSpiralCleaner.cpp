//
// Created by Joshua Bradt on 7/28/17.
//

#include "HoughSpiralCleaner.h"

namespace attpc {
namespace cleaning {

HoughSpiralCleaner::HoughSpiralCleaner(const HoughSpiralCleanerConfig& config)
: numAngleBinsToReduce(config.numAngleBinsToReduce)
, houghSpaceSliceSize(config.houghSpaceSliceSize)
, linHough(config.linearHoughNumBins, config.linearHoughMaxRadius)
, circHough(config.circularHoughNumBins, config.circularHoughMaxRadius)
{}

Eigen::ArrayXd HoughSpiralCleaner::findArcLength(const Eigen::ArrayXXd& xy, const Eigen::Vector2d center) const {
    const Eigen::ArrayXd xOffset = xy.col(0) - center(0);
    const Eigen::ArrayXd yOffset = xy.col(1) - center(1);

    Eigen::ArrayXd rads = Eigen::sqrt(xOffset.square() + yOffset.square());
    Eigen::ArrayXd thetas = Eigen::atan(yOffset / xOffset);

    return rads * thetas;
}

Eigen::Index HoughSpiralCleaner::findMaxAngleBin(const Eigen::ArrayXXd& houghSpace) const {
    // Find the ordering of indices that would sort the array
    using coefArrayType = Eigen::Array<Eigen::Index, 2, 1>;
    std::vector<coefArrayType> indices;
    for (Eigen::Index i = 0; i < houghSpace.rows(); ++i) {
        for (Eigen::Index j = 0; j < houghSpace.cols(); ++j) {
            indices.emplace_back(i, j);
        }
    }

    std::sort(indices.begin(), indices.end(), [&houghSpace](auto idxA, auto idxB) {
        return houghSpace(idxA(0), idxA(1)) < houghSpace(idxB(0), idxB(1));
    });

    // Find the mean bin from the last few elements of the sorted list
    // The zero element must be declared separately for std::accumulate to work
    const coefArrayType zeroBin = coefArrayType::Zero(2, 1);
    coefArrayType binTotal = std::accumulate(indices.end() - numAngleBinsToReduce, indices.end(), zeroBin);
    coefArrayType meanBin = Eigen::floor(binTotal / numAngleBinsToReduce);

    return meanBin(0);
}

Eigen::ArrayXd HoughSpiralCleaner::findMaxAngleSlice(const Eigen::ArrayXXd& houghSpace,
                                                     const Eigen::Index maxAngleBin) const {
    return houghSpace.block(maxAngleBin - houghSpaceSliceSize, 0, 2 * houghSpaceSliceSize, houghSpace.cols())
                     .colwise().sum();
}

}
}
