#include "HoughSpace.h"

namespace attpc {
namespace cleaning {

HoughSpace::HoughSpace(const Eigen::Index numBins_, const double maxRadiusValue_)
: numBins(numBins_)
, minRadiusValue(-maxRadiusValue_)
, maxRadiusValue(maxRadiusValue_)
, minAngleValue(0)
, maxAngleValue(M_PI)
, data(decltype(data)::Zero(numBins, numBins))
{}

auto HoughSpace::getValueAtBin(const Eigen::Index angleBin, const Eigen::Index radBin) -> ScalarType& {
    return data(angleBin, radBin);
}

auto HoughSpace::getValueAtBin(const Eigen::Index angleBin, const Eigen::Index radBin) const -> ScalarType {
    return data(angleBin, radBin);
}

auto HoughSpace::getValueAtCoords(const double angle, const double radius) -> ScalarType& {
    const Eigen::Index angleBin = findBinFromAngle(angle);
    const Eigen::Index radiusBin = findBinFromRadius(radius);

    return getValueAtBin(angleBin, radiusBin);
}

auto HoughSpace::getValueAtCoords(const double angle, const double radius) const -> ScalarType {
    const Eigen::Index angleBin = findBinFromAngle(angle);
    const Eigen::Index radiusBin = findBinFromRadius(radius);

    return getValueAtBin(angleBin, radiusBin);
}

auto HoughSpace::findMaximum() const -> ScalarType {
    return data.maxCoeff();
}

auto HoughSpace::findMaximum(Eigen::Index& maxAngleBin, Eigen::Index& maxRadBin) const -> ScalarType {
    return data.maxCoeff(&maxAngleBin, &maxRadBin);
}

auto HoughSpace::getAngularSlice(const Eigen::Index angleBin) -> DataBlockType {
    return getAngularSlice(angleBin, 1);
}

auto HoughSpace::getAngularSlice(const Eigen::Index angleBin) const -> ConstDataBlockType {
    return getAngularSlice(angleBin, 1);
}

auto HoughSpace::getAngularSlice(const Eigen::Index minAngleBin, const Eigen::Index width) -> DataBlockType {
    return data.block(minAngleBin, 0, width, numBins);
}

auto HoughSpace::getAngularSlice(const Eigen::Index minAngleBin, const Eigen::Index width) const -> ConstDataBlockType {
    return data.block(minAngleBin, 0, width, numBins);
}

auto HoughSpace::getRadialSlice(const Eigen::Index radiusBin) -> DataBlockType {
    return getRadialSlice(radiusBin, 1);
}

auto HoughSpace::getRadialSlice(const Eigen::Index radiusBin) const -> ConstDataBlockType {
    return getRadialSlice(radiusBin, 1);
}

auto HoughSpace::getRadialSlice(const Eigen::Index minRadiusBin, const Eigen::Index width) -> DataBlockType {
    return data.block(0, minRadiusBin, numBins, width);
}

auto HoughSpace::getRadialSlice(const Eigen::Index minRadiusBin, const Eigen::Index width) const -> ConstDataBlockType {
    return data.block(0, minRadiusBin, numBins, width);
}

double HoughSpace::findRadiusFromBin(const Eigen::Index bin) const {
    return findValue(bin, minRadiusValue, maxRadiusValue);
}

Eigen::Index HoughSpace::findBinFromRadius(const double radius) const {
    return findBin(radius, minRadiusValue, maxRadiusValue);
}

double HoughSpace::findAngleFromBin(const Eigen::Index bin) const {
    return findValue(bin, minAngleValue, maxAngleValue);
}

Eigen::Index HoughSpace::findBinFromAngle(const double angle) const {
    return findBin(angle, minAngleValue, maxAngleValue);
}

double HoughSpace::findBinSize(const double lowerBound, const double upperBound) const {
    return (upperBound - lowerBound) / numBins;
}

Eigen::Index HoughSpace::findBin(const double value, const double lowerBound, const double upperBound) const {
    return static_cast<Eigen::Index>(std::floor((value - lowerBound) / findBinSize(lowerBound, upperBound)));
}

double HoughSpace::findValue(const Eigen::Index bin, const double lowerBound, const double upperBound) const {
    return bin * findBinSize(lowerBound, upperBound) + lowerBound;
}

}
}
