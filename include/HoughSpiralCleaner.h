//
// Created by Joshua Bradt on 7/28/17.
//

#ifndef ATTPC_CLEANING_HOUGHSPIRALCLEANER_H
#define ATTPC_CLEANING_HOUGHSPIRALCLEANER_H

#include <Eigen/Core>
#include <vector>
#include <algorithm>
#include <numeric>
#include <array>
#include "CircularHoughTransform.h"
#include "LinearHoughTransform.h"
#include "utilities.h"

namespace attpc {
namespace cleaning {

class HoughSpiralCleanerConfig {
public:
    int linearHoughNumBins;
    int linearHoughMaxRadius;
    int circularHoughNumBins;
    int circularHoughMaxRadius;
    int numAngleBinsToReduce;
    int houghSpaceSliceSize;
    int peakWidth;
};

class HoughSpiralCleaner {
public:
    HoughSpiralCleaner(const HoughSpiralCleanerConfig& config);

    Eigen::ArrayXd findArcLength(const Eigen::ArrayXXd& xy, const Eigen::Vector2d center) const;
    Eigen::Index findMaxAngleBin(const Eigen::ArrayXXd& houghSpace) const;
    Eigen::ArrayXd findMaxAngleSlice(const Eigen::ArrayXXd& houghSpace, const Eigen::Index maxAngleBin) const;
    std::vector<double> findPeakRadiusBins(const Eigen::ArrayXd& houghSlice) const;

private:
    int numAngleBinsToReduce;
    int houghSpaceSliceSize;
    int peakWidth;

    LinearHoughTransform linHough;
    CircularHoughTransform circHough;
};

}
}

#endif //ATTPC_CLEANING_HOUGHSPIRALCLEANER_H
