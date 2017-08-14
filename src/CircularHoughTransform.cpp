//
// Created by Joshua Bradt on 7/27/17.
//

#include "CircularHoughTransform.h"

namespace attpc {
namespace cleaning {

CircularHoughTransform::CircularHoughTransform(const Eigen::Index numBins_, const double maxRadiusValue_, const Eigen::Index rowOffset_)
: HoughTransform(numBins_, maxRadiusValue_, rowOffset_)
{}

Eigen::Vector2d CircularHoughTransform::findCenter(const Eigen::ArrayXXd& data) const {
    HoughSpace hspace = findHoughSpace(data);

    Eigen::Index maxRow = 0;
    Eigen::Index maxCol = 0;
    hspace.findMaximum(maxRow, maxCol);

    // Convert max bin to angle, radius values
    const double maxAngle = hspace.findAngleFromBin(maxRow);
    const double maxRadius = hspace.findRadiusFromBin(maxCol);

    // Convert angle and radius values to positions in the data space
    Eigen::Vector2d center;
    center(0) = maxRadius * cos(maxAngle);
    center(1) = maxRadius * sin(maxAngle);

    return center;
}

Eigen::ArrayXd CircularHoughTransform::radiusFunction(const Eigen::ArrayXXd& data, const double angle) const {
    const Eigen::Index numPts = data.rows();
    if (numPts < getRowOffset()) {
        return Eigen::ArrayXd();  // Return null (empty) matrix
    }

    const auto x1 = data.block(getRowOffset(), 0, numPts - getRowOffset(), 1);
    const auto y1 = data.block(getRowOffset(), 1, numPts - getRowOffset(), 1);
    const auto x0 = data.block(0, 0, numPts - getRowOffset(), 1);
    const auto y0 = data.block(0, 1, numPts - getRowOffset(), 1);

    const auto numer = (x1*x1 - x0*x0) + (y1*y1 - y0*y0);
    const auto denom = 2 * ((x1 - x0) * std::cos(angle) + (y1 - y0) * std::sin(angle));
    return numer / denom;
}

}
}
