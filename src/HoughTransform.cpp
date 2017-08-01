//
// Created by Joshua Bradt on 7/27/17.
//

#include "HoughTransform.h"

namespace attpc {
namespace cleaning {

HoughTransform::HoughTransform(const int numBins_, const int maxRadiusValue_, const int rowOffset_)
: numBins(numBins_)
, rowOffset(rowOffset_)
, minRadiusValue(-maxRadiusValue_)
, maxRadiusValue(maxRadiusValue_)
, minAngleValue(0)
, maxAngleValue(M_PI)
{}

Eigen::ArrayXXd HoughTransform::findHoughSpace(const Eigen::ArrayXXd& data) const {
    Eigen::ArrayXXd houghSpace = Eigen::ArrayXXd::Zero(numBins, numBins);

    for (int angleBin = 0; angleBin < numBins; angleBin++) {
        double angle = findAngleFromBin(angleBin);
        // Precompute sin and cos here so they aren't done nrows times for each angle
        double cosAngle = std::cos(angle);
        double sinAngle = std::sin(angle);

        for (int xyz_idx = rowOffset; xyz_idx < data.rows(); xyz_idx++) {
            double rad = radiusFunction(data, xyz_idx, cosAngle, sinAngle);
            if (rad >= minRadiusValue && rad < maxRadiusValue) {
                // Find and increment histogram/accumulator bin corresponding to rad
                Eigen::Index radBin = findBinFromRadius(rad);
                houghSpace(angleBin, radBin) += 1;  // TODO: Check if this is actually thread-safe...
            }
        }
    }

    return houghSpace;
}

double HoughTransform::findRadiusFromBin(const Eigen::Index bin) const {
    return findValue(bin, minRadiusValue, maxRadiusValue);
}

Eigen::Index HoughTransform::findBinFromRadius(const double radius) const {
    return findBin(radius, minRadiusValue, maxRadiusValue);
}

double HoughTransform::findAngleFromBin(const Eigen::Index bin) const {
    return findValue(bin, minAngleValue, maxAngleValue);
}

Eigen::Index HoughTransform::findBinFromAngle(const double angle) const {
    return findBin(angle, minAngleValue, maxAngleValue);
}

double HoughTransform::findBinSize(const double lowerBound, const double upperBound) const {
    return (upperBound - lowerBound) / numBins;
}

Eigen::Index HoughTransform::findBin(const double value, const double lowerBound, const double upperBound) const {
    return (value - lowerBound) / findBinSize(lowerBound, upperBound);
}

double HoughTransform::findValue(const Eigen::Index bin, const double lowerBound, const double upperBound) const {
    return bin * findBinSize(lowerBound, upperBound) + lowerBound;
}

}
}
