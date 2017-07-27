//
// Created by Joshua Bradt on 7/27/17.
//

#include "LinearHoughTransform.h"


namespace attpc {
namespace cleaning {

LinearHoughTransform::LinearHoughTransform(const int numBins_, const int maxRadiusValue_)
: HoughTransform(numBins_, maxRadiusValue_)
{}

double LinearHoughTransform::radiusFunction(const Eigen::ArrayXXd& data, const Eigen::Index rowIdx,
                                            const double costh, const double sinth) const {
    const double x = data(rowIdx, 0);
    const double y = data(rowIdx, 1);
    return x * costh + y * sinth;
}


}
}
