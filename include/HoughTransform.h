//
// Created by Joshua Bradt on 7/27/17.
//

#ifndef ATTPC_CLEANING_HOUGHTRANSFORM_H
#define ATTPC_CLEANING_HOUGHTRANSFORM_H

#include <cstddef>
#include <Eigen/Core>

namespace attpc {
namespace cleaning {

class HoughTransform {
public:
    HoughTransform(const int numBins_, const int maxRadiusValue_);

    Eigen::ArrayXXd findHoughSpace(const Eigen::ArrayXXd& data) const;

    virtual double findRadiusFromBin(const Eigen::Index bin) const;
    virtual Eigen::Index findBinFromRadius(const double radius) const;
    
    virtual double findAngleFromBin(const Eigen::Index bin) const;
    virtual Eigen::Index findBinFromAngle(const double angle) const;


protected:
    virtual double radiusFunction(const Eigen::ArrayXXd& data, const Eigen::Index rowIdx,
                                  const double costh, const double sinth) const = 0;

    int numBins;
    int maxRadiusValue;
    int rowOffset;
};

}
}


#endif //ATTPC_CLEANING_HOUGHTRANSFORM_H
