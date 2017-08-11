//
// Created by Joshua Bradt on 7/28/17.
//

#ifndef ATTPC_CLEANING_HOUGHSPIRALCLEANER_H
#define ATTPC_CLEANING_HOUGHSPIRALCLEANER_H

#include <Eigen/Core>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cassert>
#include <array>
#include "CircularHoughTransform.h"
#include "LinearHoughTransform.h"
#include "HoughSpace.h"
#include "utilities.h"

namespace attpc {
namespace cleaning {

class HoughSpiralCleanerConfig {
public:
    Eigen::Index linearHoughNumBins;
    double linearHoughMaxRadius;
    Eigen::Index circularHoughNumBins;
    double circularHoughMaxRadius;
    Eigen::Index numAngleBinsToReduce;
    Eigen::Index houghSpaceSliceSize;
    Eigen::Index peakWidth;
};

class HoughSpiralCleaner {
public:
    using AngleSliceArrayType = Eigen::Array<HoughSpace::ScalarType, Eigen::Dynamic, 1>;

    HoughSpiralCleaner(const HoughSpiralCleanerConfig& config);

    Eigen::ArrayXd findArcLength(const Eigen::ArrayXXd& xy, const Eigen::Vector2d center) const;
    Eigen::Index findMaxAngleBin(const HoughSpace& houghSpace) const;
    AngleSliceArrayType findMaxAngleSlice(const HoughSpace& houghSpace, const Eigen::Index maxAngleBin) const;
    std::vector<double> findPeakRadiusBins(const AngleSliceArrayType& houghSlice) const;

private:
    Eigen::Index numAngleBinsToReduce;
    Eigen::Index houghSpaceSliceSize;
    Eigen::Index peakWidth;

    LinearHoughTransform linHough;
    CircularHoughTransform circHough;
};

}
}

#endif //ATTPC_CLEANING_HOUGHSPIRALCLEANER_H
