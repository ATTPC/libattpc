//
// Created by Joshua Bradt on 7/27/17.
//

#ifndef ATTPC_CLEANING_LINEARHOUGHTRANSFORM_H
#define ATTPC_CLEANING_LINEARHOUGHTRANSFORM_H

#include "HoughTransform.h"
#include <Eigen/Core>

namespace attpc {
namespace cleaning {

class LinearHoughTransform : public HoughTransform {
public:
    LinearHoughTransform(const Eigen::Index numBins_, const double maxRadiusValue_);

protected:
    Eigen::ArrayXd radiusFunction(const Eigen::ArrayXXd& data, const double angle) const override;
};

}
}




#endif //ATTPC_CLEANING_LINEARHOUGHTRANSFORM_H
