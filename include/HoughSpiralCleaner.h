//
// Created by Joshua Bradt on 7/28/17.
//

#ifndef ATTPC_CLEANING_HOUGHSPIRALCLEANER_H
#define ATTPC_CLEANING_HOUGHSPIRALCLEANER_H

#include <Eigen/Core>
#include <vector>
#include <algorithm>
#include <numeric>
#include <limits>
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
    int minPointsPerLine;
};

class HoughSpiralCleanerResult {
public:
    HoughSpiralCleanerResult() = default;
    HoughSpiralCleanerResult(const Eigen::Index numPts);

    Eigen::Array<Eigen::Index, Eigen::Dynamic, 1> labels;
    Eigen::ArrayXd distancesToNearestLine;
};

class HoughSpiralCleaner {
public:
    using AngleSliceArrayType = Eigen::Array<HoughSpace::ScalarType, Eigen::Dynamic, 1>;

    HoughSpiralCleaner(const HoughSpiralCleanerConfig& config);

    HoughSpiralCleanerResult processEvent(const Eigen::ArrayXXd& xyz) const;

    Eigen::ArrayXd findArcLength(const Eigen::ArrayXXd& xy, const Eigen::Vector2d center) const;
    HoughSpace findHoughSpace(const Eigen::ArrayXd& zs, const Eigen::ArrayXd& arclens) const;
    Eigen::Index findMaxAngleBin(const HoughSpace& houghSpace) const;
    AngleSliceArrayType findMaxAngleSlice(const HoughSpace& houghSpace, const Eigen::Index maxAngleBin) const;
    std::vector<double> findPeakRadiusBins(const AngleSliceArrayType& houghSlice) const;
    HoughSpiralCleanerResult classifyPoints(const Eigen::ArrayXd& zs, const Eigen::ArrayXd& arclens,
                                            const double maxAngle, const Eigen::ArrayXd& radii) const;

private:
    Eigen::Index numAngleBinsToReduce;
    Eigen::Index houghSpaceSliceSize;
    Eigen::Index peakWidth;
    int minPointsPerLine;

    LinearHoughTransform linHough;
    CircularHoughTransform circHough;
};

}
}

#endif //ATTPC_CLEANING_HOUGHSPIRALCLEANER_H
