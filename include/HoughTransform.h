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
    HoughTransform(const int numBins_, const int maxRadiusValue_, const int rowOffset_ = 0);

    Eigen::ArrayXXd findHoughSpace(const Eigen::ArrayXXd& data) const;

    double findRadiusFromBin(const Eigen::Index bin) const;
    Eigen::Index findBinFromRadius(const double radius) const;

    double findAngleFromBin(const Eigen::Index bin) const;
    Eigen::Index findBinFromAngle(const double angle) const;

    int getNumBins() const { return numBins; }
    int getRowOffset() const { return rowOffset; }
    double getAngleBinSize() const { return findBinSize(minAngleValue, maxAngleValue); }
    double getRadiusBinSize() const { return findBinSize(minRadiusValue, maxRadiusValue); }
    double getMinRadiusValue() const { return minRadiusValue; }
    double getMaxRadiusValue() const { return maxRadiusValue; }
    double getMinAngleValue() const { return minAngleValue; }
    double getMaxAngleValue() const { return maxAngleValue; }

protected:
    virtual double radiusFunction(const Eigen::ArrayXXd& data, const Eigen::Index rowIdx,
                                  const double costh, const double sinth) const = 0;

private:
    double findBinSize(const double lowerBound, const double upperBound) const;
    Eigen::Index findBin(const double value, const double lowerBound, const double upperBound) const;
    double findValue(const Eigen::Index bin, const double lowerBound, const double upperBound) const;

    int numBins;
    int rowOffset;

    double minRadiusValue;
    double maxRadiusValue;

    double minAngleValue;
    double maxAngleValue;
};

}
}


#endif //ATTPC_CLEANING_HOUGHTRANSFORM_H
