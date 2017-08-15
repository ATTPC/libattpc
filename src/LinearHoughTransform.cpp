//
// Created by Joshua Bradt on 7/27/17.
//

#include "LinearHoughTransform.h"


namespace attpc {
namespace cleaning {

LinearHoughTransform::LinearHoughTransform(const Eigen::Index numBins_, const double maxRadiusValue_)
: HoughTransform(numBins_, maxRadiusValue_)
{}

Eigen::ArrayXd
LinearHoughTransform::radiusFunction(const Eigen::Ref<const Eigen::ArrayXd>& xs,
                                     const Eigen::Ref<const Eigen::ArrayXd>& ys,
                                     const double angle) const {
    return xs * std::cos(angle) + ys * std::sin(angle);
}


}
}
