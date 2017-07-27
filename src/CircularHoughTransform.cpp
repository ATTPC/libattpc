//
// Created by Joshua Bradt on 7/27/17.
//

#include "CircularHoughTransform.h"

namespace attpc {
namespace cleaning {

CircularHoughTransform::CircularHoughTransform(const int numBins_, const int maxRadiusValue_, const int rowOffset_)
: HoughTransform(numBins_, maxRadiusValue_)
{
    rowOffset = rowOffset_;
}

double CircularHoughTransform::radiusFunction(const Eigen::ArrayXXd& data, const Eigen::Index rowIdx,
                                              const double costh, const double sinth) const {
    const double x1 = data(rowIdx, 0);
    const double y1 = data(rowIdx, 1);
    const double x0 = data(rowIdx - rowOffset, 0);
    const double y0 = data(rowIdx - rowOffset, 1);

    const double numer = (x1*x1 - x0*x0) + (y1*y1 - y0*y0);
    const double denom = 2 * ((x1 - x0) * costh + (y1 - y0) * sinth);
    return numer / denom;
}

}
}