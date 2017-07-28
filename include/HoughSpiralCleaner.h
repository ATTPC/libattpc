//
// Created by Joshua Bradt on 7/28/17.
//

#ifndef ATTPC_CLEANING_HOUGHSPIRALCLEANER_H
#define ATTPC_CLEANING_HOUGHSPIRALCLEANER_H

#include <Eigen/Core>
#include <vector>
#include <algorithm>
#include <numeric>
#include "CircularHoughTransform.h"
#include "LinearHoughTransform.h"

namespace attpc {
namespace cleaning {

class HoughSpiralCleaner {
public:
    HoughSpiralCleaner();

    Eigen::ArrayXd findArcLength(const Eigen::Array2Xd& xy, const Eigen::Vector2d center) const;
    Eigen::Index findMaxAngleBin(const Eigen::ArrayXXd& houghSpace) const;
    Eigen::ArrayXd findMaxAngleSlice(const Eigen::ArrayXXd& houghSpace, const Eigen::Index maxAngleBin) const;

private:
    int numAngleBinsToReduce;
    int houghSpaceSliceSize;
};

}
}

#endif //ATTPC_CLEANING_HOUGHSPIRALCLEANER_H
