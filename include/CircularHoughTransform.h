//
// Created by Joshua Bradt on 7/27/17.
//

#ifndef ATTPC_CLEANING_CIRCULARHOUGHTRANSFORM_H
#define ATTPC_CLEANING_CIRCULARHOUGHTRANSFORM_H

#include "HoughTransform.h"

namespace attpc {
namespace cleaning {

class CircularHoughTransform : public HoughTransform {
public:
    CircularHoughTransform(const Eigen::Index numBins_, const double maxRadiusValue_, const Eigen::Index rowOffset_ = 5);

    Eigen::Vector2d findCenter(const Eigen::ArrayXXd& data) const;

protected:
    double radiusFunction(const Eigen::ArrayXXd& data, const Eigen::Index rowIdx, const double costh,
                          const double sinth) const override;
};

}
}

#endif //ATTPC_CLEANING_CIRCULARHOUGHTRANSFORM_H
