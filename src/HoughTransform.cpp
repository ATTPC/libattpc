//
// Created by Joshua Bradt on 7/27/17.
//

#include "HoughTransform.h"

namespace attpc {
namespace cleaning {

HoughTransform::HoughTransform(const int numBins_, const int maxRadiusValue_)
: numBins(numBins_)
, maxRadiusValue(maxRadiusValue_)
{}

Eigen::ArrayXXd HoughTransform::findHoughSpace(const Eigen::ArrayXXd& data) const {
    const double thetaStep = M_PI / numBins;  // Size of theta (angle) bins
    const double minRadiusValue = -maxRadiusValue;

    Eigen::ArrayXXd result = Eigen::ArrayXXd::Zero(numBins, numBins);

    #pragma omp parallel for
    for (int thetaIdx = 0; thetaIdx < numBins; thetaIdx++) {
        double theta = thetaIdx * thetaStep;
        // Precompute sin and cos here so they aren't done nrows times for each theta
        double cosTheta = std::cos(theta);
        double sinTheta = std::sin(theta);

        for (int xyz_idx = rowOffset; xyz_idx < data.rows(); xyz_idx++) {
            double rad = radiusFunction(data, xyz_idx, cosTheta, sinTheta);
            if (rad >= minRadiusValue && rad < maxRadiusValue) {
                // Find and increment histogram/accumulator bin corresponding to rad
                Eigen::Index radBin = findBinFromRadius(rad);
                result(thetaIdx, radBin) += 1;  // TODO: Check if this is actually thread-safe...
            }
        }
    }

    return result;
}

double HoughTransform::findRadiusFromBin(const Eigen::Index bin) const {
    return bin * 2 * maxRadiusValue / numBins - maxRadiusValue;
}

Eigen::Index HoughTransform::findBinFromRadius(const double radius) const {
    return std::floor((radius + maxRadiusValue) * numBins / (2 * maxRadiusValue));
}

double HoughTransform::findAngleFromBin(const Eigen::Index bin) const {
    return bin * M_PI / numBins;
}

Eigen::Index HoughTransform::findBinFromAngle(const double angle) const {
    return angle * numBins / M_PI;
}

}
}