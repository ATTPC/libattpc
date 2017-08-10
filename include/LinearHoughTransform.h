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
    double radiusFunction(const Eigen::ArrayXXd& data, const Eigen::Index rowIdx, const double costh,
                          const double sinth) const override;
};

}
}




#endif //ATTPC_CLEANING_LINEARHOUGHTRANSFORM_H
