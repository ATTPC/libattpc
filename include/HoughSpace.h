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

    HoughSpace(const Eigen::Index numBins_, const double maxRadiusValue_);

    ScalarType& getValueAtBin(const Eigen::Index angleBin, const Eigen::Index radBin);
    ScalarType  getValueAtBin(const Eigen::Index angleBin, const Eigen::Index radBin) const;

    ScalarType& getValueAtCoords(const double angle, const double radius);
    ScalarType  getValueAtCoords(const double angle, const double radius) const;

    ScalarType findMaximum() const;
    ScalarType findMaximum(Eigen::Index& maxAngleBin, Eigen::Index& maxRadBin) const;

    DataBlockType getAngularSlice(const Eigen::Index angleBin);
    ConstDataBlockType getAngularSlice(const Eigen::Index angleBin) const;
    DataBlockType getAngularSlice(const Eigen::Index minAngleBin, const Eigen::Index width);
    ConstDataBlockType getAngularSlice(const Eigen::Index minAngleBin, const Eigen::Index width) const;

    DataBlockType getRadialSlice(const Eigen::Index radiusBin);
    ConstDataBlockType getRadialSlice(const Eigen::Index radiusBin) const;
    DataBlockType getRadialSlice(const Eigen::Index minRadiusBin, const Eigen::Index width);
    ConstDataBlockType getRadialSlice(const Eigen::Index minRadiusBin, const Eigen::Index width) const;


    /**
     * Find the radius value corresponding to the given bin.
     * @param  bin The bin number
     * @return     The radius corresponding to the lower bound of this bin.
     */
    double findRadiusFromBin(const Eigen::Index bin) const;

    /**
     * Find the bin that contains the given radius.
     * @param  radius The radius value.
     * @return        The bin number that contains this radius.
     */
    Eigen::Index findBinFromRadius(const double radius) const;

    /**
     * Find the angle corresponding to the given bin.
     * @param  bin The bin number.
     * @return     The angle corresponding to the lower bound of this bin.
     */
    double findAngleFromBin(const Eigen::Index bin) const;

    /**
     * Find the bin corresponding to the given angle.
     * @param  angle The angle value.
     * @return       The bin number that contains this angle.
     */
    Eigen::Index findBinFromAngle(const double angle) const;

    //! Get the number of bins.
    Eigen::Index getNumBins() const { return numBins; }

    //! Get the size of one angle bin.
    double getAngleBinSize() const { return findBinSize(minAngleValue, maxAngleValue); }

    //! Get the size of one radius bin.
    double getRadiusBinSize() const { return findBinSize(minRadiusValue, maxRadiusValue); }

    //! Get the radius corresponding to the lower bound of the first bin.
    double getMinRadiusValue() const { return minRadiusValue; }

    //! Get the radius corresponding to the upper bound of the last bin.
    double getMaxRadiusValue() const { return maxRadiusValue; }

    //! Get the angle corresponding to the lower bound of the first bin.
    double getMinAngleValue() const { return minAngleValue; }

    //! Get the angle corresponding to the upper bound of the last bin.
    double getMaxAngleValue() const { return maxAngleValue; }

private:
    /**
     * Calculate the size of a bin using the provided bounds and the numBins attribute of the class.
     *
     * @param  lowerBound The lower bound of the smallest bin.
     * @param  upperBound The upper bound of the largest bin.
     * @return            The size of one bin.
     */
    double findBinSize(const double lowerBound, const double upperBound) const;

    /**
     * Calculate the bin number corresponding to a given value based on the provided bounds and the number of
     * bins defined in the class.
     *
     * @param  value      The value of the parameter being binned.
     * @param  lowerBound The lower bound of the smallest bin.
     * @param  upperBound The upper bound of the largest bin.
     * @return            The corresponding bin number.
     */
    Eigen::Index findBin(const double value, const double lowerBound, const double upperBound) const;

    /**
     * Find the value corresponding to the lower bound of the given bin using the provided bounds and the number
     * of bins defined in the class.
     *
     * @param  bin        The bin number.
     * @param  lowerBound The lower bound of the smallest bin.
     * @param  upperBound The upper bound of the largest bin.
     * @return            The value corresponding to the lower bound of the given bin.
     */
    double findValue(const Eigen::Index bin, const double lowerBound, const double upperBound) const;

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
