//
// Created by Joshua Bradt on 7/27/17.
//

#include "attpc/cleaning/CircularHoughTransform.h"

namespace attpc {
namespace cleaning {

CircularHoughTransform::CircularHoughTransform(const Eigen::Index numBins_, const double maxRadiusValue_, const Eigen::Index rowOffset_)
: HoughTransform(numBins_, maxRadiusValue_, rowOffset_)
{}

Eigen::Vector2d CircularHoughTransform::findCenter(const Eigen::Ref<const Eigen::ArrayXd>& xs,
                                                   const Eigen::Ref<const Eigen::ArrayXd>& ys) const {
    HoughSpace hspace = findHoughSpace(xs, ys);

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

Eigen::ArrayXd
CircularHoughTransform::radiusFunction(const Eigen::Ref<const Eigen::ArrayXd>& xs,
                                       const Eigen::Ref<const Eigen::ArrayXd>& ys,
                                       const double angle) const {
    const Eigen::Index numPts = xs.rows();
    if (numPts < getRowOffset()) {
        return Eigen::ArrayXd();  // Return null (empty) matrix
    }

    const auto x1 = xs.segment(getRowOffset(), numPts - getRowOffset());
    const auto y1 = ys.segment(getRowOffset(), numPts - getRowOffset());
    const auto x0 = xs.segment(0, numPts - getRowOffset());
    const auto y0 = ys.segment(0, numPts - getRowOffset());

    const auto numer = (x1*x1 - x0*x0) + (y1*y1 - y0*y0);
    const auto denom = 2 * ((x1 - x0) * std::cos(angle) + (y1 - y0) * std::sin(angle));
    return numer / denom;
}

}
}
