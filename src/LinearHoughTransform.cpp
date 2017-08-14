//
// Created by Joshua Bradt on 7/27/17.
//

#include "LinearHoughTransform.h"


namespace attpc {
namespace cleaning {

LinearHoughTransform::LinearHoughTransform(const Eigen::Index numBins_, const double maxRadiusValue_)
: HoughTransform(numBins_, maxRadiusValue_)
{}

Eigen::ArrayXd LinearHoughTransform::radiusFunction(const Eigen::ArrayXXd& data, const double angle) const {
    const auto x = data.col(0);
    const auto y = data.col(1);
    return x * std::cos(angle) + y * std::sin(angle);
}


}
}
