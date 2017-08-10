//
// Created by Joshua Bradt on 7/28/17.
//

#include "HoughSpiralCleaner.h"

namespace attpc {
namespace cleaning {

HoughSpiralCleaner::HoughSpiralCleaner(const HoughSpiralCleanerConfig& config)
: numAngleBinsToReduce(config.numAngleBinsToReduce)
, houghSpaceSliceSize(config.houghSpaceSliceSize)
, peakWidth(config.peakWidth)
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

Eigen::Index HoughSpiralCleaner::findMaxAngleBin(const HoughSpace& houghSpace) const {
    // Find the ordering of indices that would sort the array
    using coefArrayType = Eigen::Array<Eigen::Index, 2, 1>;
    std::vector<coefArrayType> indices;
    for (Eigen::Index i = 0; i < houghSpace.getNumBins(); ++i) {
        for (Eigen::Index j = 0; j < houghSpace.getNumBins(); ++j) {
            indices.emplace_back(i, j);
        }
    }

    std::sort(indices.begin(), indices.end(), [&houghSpace](auto idxA, auto idxB) {
        return houghSpace.getValueAtBin(idxA(0), idxA(1)) < houghSpace.getValueAtBin(idxB(0), idxB(1));
    });

    // Find the mean bin from the last few elements of the sorted list
    // The zero element must be declared separately for std::accumulate to work
    const coefArrayType zeroBin = coefArrayType::Zero(2, 1);
    coefArrayType binTotal = std::accumulate(indices.end() - numAngleBinsToReduce, indices.end(), zeroBin);
    coefArrayType meanBin = Eigen::floor(binTotal / numAngleBinsToReduce);

    return meanBin(0);
}

Eigen::Array<HoughSpace::ScalarType, Eigen::Dynamic, Eigen::Dynamic>
    HoughSpiralCleaner::findMaxAngleSlice(const HoughSpace& houghSpace, const Eigen::Index maxAngleBin) const {
    return houghSpace.getAngularSlice(maxAngleBin - houghSpaceSliceSize, 2 * houghSpaceSliceSize)
                     .colwise().sum();
}

std::vector<double> HoughSpiralCleaner::findPeakRadiusBins(const Eigen::ArrayXd& houghSlice) const {
    const std::vector<Eigen::Index> maxLocs = findPeakLocations(houghSlice, 2);
    std::vector<double> peakCtrs;

    for (const Eigen::Index pkIdx : maxLocs) {
        const Eigen::Index firstPt = std::max(pkIdx - peakWidth, Eigen::Index{0});
        const Eigen::Index lastPt = std::max(pkIdx + peakWidth, houghSlice.rows() - 1);

        const Eigen::VectorXd positions = Eigen::VectorXd::LinSpaced(lastPt - firstPt + 1, firstPt, lastPt);
        const Eigen::VectorXd values = houghSlice.segment(firstPt, lastPt - firstPt + 1);
        const double peakCtrOfGrav = positions.dot(values) / values.sum();

        peakCtrs.push_back(peakCtrOfGrav);
    }

    return peakCtrs;
}

}
}
