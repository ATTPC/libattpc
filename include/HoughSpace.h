#ifndef ATTPC_CLEANING_HOUGHSPACE_H
#define ATTPC_CLEANING_HOUGHSPACE_H

#include <Eigen/Core>
#include <cassert>

namespace attpc {
namespace cleaning {

class HoughSpace {
public:
    using ScalarType = long;  // has at least 32 bits by standard
    using DataArrayType = Eigen::Array<ScalarType, Eigen::Dynamic, Eigen::Dynamic>;
    using DataBlockType = Eigen::Block<DataArrayType>;
    using ConstDataBlockType = const Eigen::Block<const DataArrayType>;

    HoughSpace(const Eigen::Index numBins_, const double maxRadiusValue_)
    : numBins(numBins_)
    , minRadiusValue(-maxRadiusValue_)
    , maxRadiusValue(maxRadiusValue_)
    , minAngleValue(0)
    , maxAngleValue(M_PI)
    , data(decltype(data)::Zero(numBins, numBins))
    {}

    inline ScalarType& getValueAtBin(const Eigen::Index angleBin, const Eigen::Index radBin) {
        return data(angleBin, radBin);
    }
    inline ScalarType  getValueAtBin(const Eigen::Index angleBin, const Eigen::Index radBin) const {
        return data(angleBin, radBin);
    }

    inline ScalarType& getValueAtCoords(const double angle, const double radius) {
        const Eigen::Index angleBin = findBinFromAngle(angle);
        const Eigen::Index radiusBin = findBinFromRadius(radius);

        return getValueAtBin(angleBin, radiusBin);
    }
    inline ScalarType  getValueAtCoords(const double angle, const double radius) const {
        const Eigen::Index angleBin = findBinFromAngle(angle);
        const Eigen::Index radiusBin = findBinFromRadius(radius);

        return getValueAtBin(angleBin, radiusBin);
    }

    inline ScalarType findMaximum() const { return data.maxCoeff(); }
    inline ScalarType findMaximum(Eigen::Index& maxAngleBin, Eigen::Index& maxRadBin) const {
        return data.maxCoeff(&maxAngleBin, &maxRadBin);
    }

    inline DataBlockType getAngularSlice(const Eigen::Index angleBin) { return getAngularSlice(angleBin, 1); }
    inline ConstDataBlockType getAngularSlice(const Eigen::Index angleBin) const {
        return getAngularSlice(angleBin, 1);
    }
    inline DataBlockType getAngularSlice(const Eigen::Index minAngleBin, const Eigen::Index width) {
        return data.block(minAngleBin, 0, width, numBins);
    }
    inline ConstDataBlockType getAngularSlice(const Eigen::Index minAngleBin, const Eigen::Index width) const {
        return data.block(minAngleBin, 0, width, numBins);
    }

    inline DataBlockType getRadialSlice(const Eigen::Index radiusBin) {
        return getRadialSlice(radiusBin, 1);
    }
    inline ConstDataBlockType getRadialSlice(const Eigen::Index radiusBin) const {
        return getRadialSlice(radiusBin, 1);
    }
    inline DataBlockType getRadialSlice(const Eigen::Index minRadiusBin, const Eigen::Index width) {
        return data.block(0, minRadiusBin, numBins, width);
    }
    inline ConstDataBlockType getRadialSlice(const Eigen::Index minRadiusBin, const Eigen::Index width) const {
        return data.block(0, minRadiusBin, numBins, width);
    }


    /**
     * Find the radius value corresponding to the given bin.
     * @param  bin The bin number
     * @return     The radius corresponding to the lower bound of this bin.
     */
    inline double findRadiusFromBin(const Eigen::Index bin) const {
        return findValue(bin, minRadiusValue, maxRadiusValue);
    }

    /**
     * Find the bin that contains the given radius.
     * @param  radius The radius value.
     * @return        The bin number that contains this radius.
     */
    inline Eigen::Index findBinFromRadius(const double radius) const {
        return findBin(radius, minRadiusValue, maxRadiusValue);
    }

    /**
     * Find the angle corresponding to the given bin.
     * @param  bin The bin number.
     * @return     The angle corresponding to the lower bound of this bin.
     */
    inline double findAngleFromBin(const Eigen::Index bin) const {
        return findValue(bin, minAngleValue, maxAngleValue);
    }

    /**
     * Find the bin corresponding to the given angle.
     * @param  angle The angle value.
     * @return       The bin number that contains this angle.
     */
    inline Eigen::Index findBinFromAngle(const double angle) const {
        return findBin(angle, minAngleValue, maxAngleValue);
    }

    //! Get the number of bins.
    inline Eigen::Index getNumBins() const { return numBins; }

    //! Get the size of one angle bin.
    inline double getAngleBinSize() const { return findBinSize(minAngleValue, maxAngleValue); }

    //! Get the size of one radius bin.
    inline double getRadiusBinSize() const { return findBinSize(minRadiusValue, maxRadiusValue); }

    //! Get the radius corresponding to the lower bound of the first bin.
    inline double getMinRadiusValue() const { return minRadiusValue; }

    //! Get the radius corresponding to the upper bound of the last bin.
    inline double getMaxRadiusValue() const { return maxRadiusValue; }

    //! Get the angle corresponding to the lower bound of the first bin.
    inline double getMinAngleValue() const { return minAngleValue; }

    //! Get the angle corresponding to the upper bound of the last bin.
    inline double getMaxAngleValue() const { return maxAngleValue; }

private:
    /**
     * Calculate the size of a bin using the provided bounds and the numBins attribute of the class.
     *
     * @param  lowerBound The lower bound of the smallest bin.
     * @param  upperBound The upper bound of the largest bin.
     * @return            The size of one bin.
     */
    inline double findBinSize(const double lowerBound, const double upperBound) const {
        return (upperBound - lowerBound) / numBins;
    }

    /**
     * Calculate the bin number corresponding to a given value based on the provided bounds and the number of
     * bins defined in the class.
     *
     * @param  value      The value of the parameter being binned.
     * @param  lowerBound The lower bound of the smallest bin.
     * @param  upperBound The upper bound of the largest bin.
     * @return            The corresponding bin number.
     */
    inline Eigen::Index findBin(const double value, const double lowerBound, const double upperBound) const {
        return static_cast<Eigen::Index>(std::floor((value - lowerBound) / findBinSize(lowerBound, upperBound)));
    }

    /**
     * Find the value corresponding to the lower bound of the given bin using the provided bounds and the number
     * of bins defined in the class.
     *
     * @param  bin        The bin number.
     * @param  lowerBound The lower bound of the smallest bin.
     * @param  upperBound The upper bound of the largest bin.
     * @return            The value corresponding to the lower bound of the given bin.
     */
    inline double findValue(const Eigen::Index bin, const double lowerBound, const double upperBound) const {
        return bin * findBinSize(lowerBound, upperBound) + lowerBound;
    }

private:
    //! The number of bins to use for the Hough space in both dimensions.
    Eigen::Index numBins;

    //! The lower bound of the Hough space in the radius dimension.
    double minRadiusValue;

    //! The upper bound of the Hough space in the radius dimension.
    double maxRadiusValue;

    //! The lower bound of the Hough space in the angle dimension.
    double minAngleValue;

    //! The upper bound of the Hough space in the angle dimension.
    double maxAngleValue;

    //! The Hough space data
    DataArrayType data;
};

}
}

#endif /* end of include guard: ATTPC_CLEANING_HOUGHSPACE_H */
