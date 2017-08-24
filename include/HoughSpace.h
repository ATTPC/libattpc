#ifndef ATTPC_CLEANING_HOUGHSPACE_H
#define ATTPC_CLEANING_HOUGHSPACE_H

#include "eigen_common.h"
#include <cassert>
#include <type_traits>
#include <numeric>

namespace attpc {
namespace cleaning {

/**
 * @brief A container for the calculated Hough space.
 *
 * This class holds the result of the Hough space calculation. Its main advantage over a plain matrix is that
 * it can also calculate bin numbers from radius and angle values, and it abstracts away the need to know which
 * variable corresponds to rows vs. columns in the matrix.
 *
 * The Hough space is stored internally in a square array of integers with dimension numBins. In the angle
 * dimension, the bins are bounded by 0 on the lower end and pi on the upper end. The radius dimension covers
 * the range from -maxRadiusValue to +maxRadiusValue. An important note is that the upper boundary of the range
 * is defined by the *upper* bound of the largest bin, whereas the lower boundary of the range is the *lower* bound
 * of the smallest bin.
 */
class HoughSpace {
public:
    using ScalarType = long;  // has at least 32 bits by standard
    using DataArrayType = Eigen::Array<ScalarType, Eigen::Dynamic, Eigen::Dynamic>;
    using DataBlockType = Eigen::Block<DataArrayType>;
    using ConstDataBlockType = const Eigen::Block<const DataArrayType>;

    /**
     * Constructor
     * @param numBins_        The number of bins to use in the Hough space array in each dimension. This array
     *                        will be square.
     * @param maxRadiusValue_ The radius value corresponding to the largest and smallest radius bins.
     */
    HoughSpace(const Eigen::Index numBins_, const double maxRadiusValue_)
    : numBins(numBins_)
    , minRadiusValue(-maxRadiusValue_)
    , maxRadiusValue(maxRadiusValue_)
    , minAngleValue(0)
    , maxAngleValue(M_PI)
    , data(decltype(data)::Zero(numBins, numBins))
    {}

    /**
     * @brief Get a reference to the value in a given bin.
     *
     * The return value can be used on the left-hand side of an assignment to set the value in a bin.
     *
     * Note that bounds checking is not explicitly performed by this function.
     *
     * @param  angleBin The bin in the angle dimension.
     * @param  radBin   The bin in the radius dimension.
     * @return          The value in the given bin.
     */
    inline ScalarType& getValueAtBin(const Eigen::Index angleBin, const Eigen::Index radBin) {
        return data(angleBin, radBin);
    }

    /**
     * @brief Get the value in a given bin.
     *
     * Note that bounds checking is not explicitly performed by this function.
     *
     * @param  angleBin The bin in the angle dimension.
     * @param  radBin   The bin in the radius dimension.
     * @return          The value in the given bin.
     */
    inline ScalarType  getValueAtBin(const Eigen::Index angleBin, const Eigen::Index radBin) const {
        return data(angleBin, radBin);
    }

    /**
     * @brief Get a reference the value at a given set of (radius, angle) values.
     *
     * This works like getValueAtBin, but it transforms the values into bins for you.
     *
     * The return value can be used on the left-hand side of an assignment to set the value in a bin.
     *
     * Note that bounds checking is not explicitly performed by this function.
     *
     * @param  angle  The angle value of interest. This will be binned automatically.
     * @param  radius The radius value of interest. This will be binned automatically.
     * @return        A reference to the value in the given bin.
     */
    inline ScalarType& getValueAtCoords(const double angle, const double radius) {
        const Eigen::Index angleBin = findBinFromAngle(angle);
        const Eigen::Index radiusBin = findBinFromRadius(radius);

        return getValueAtBin(angleBin, radiusBin);
    }

    /**
     * @brief Get the value at a given set of (radius, angle) values.
     *
     * This works like getValueAtBin, but it transforms the values into bins for you.
     *
     * Note that bounds checking is not explicitly performed by this function.
     *
     * @param  angle  The angle value of interest. This will be binned automatically.
     * @param  radius The radius value of interest. This will be binned automatically.
     * @return        The value in the given bin.
     */
    inline ScalarType  getValueAtCoords(const double angle, const double radius) const {
        const Eigen::Index angleBin = findBinFromAngle(angle);
        const Eigen::Index radiusBin = findBinFromRadius(radius);

        return getValueAtBin(angleBin, radiusBin);
    }

    /**
     * @brief Find the maximum value in the Hough space.
     * @return The value in the maximum bin.
     */
    inline ScalarType findMaximum() const { return data.maxCoeff(); }

    /**
     * @brief Find the maximum value and bins in the Hough space.
     *
     * The bin values are returned by reference through the two parameters given to this function.
     *
     * @param[out]  maxAngleBin The max angle bin.
     * @param[out]  maxRadBin   The max radius bin.
     * @return                  The value in the maximum bin.
     */
    inline ScalarType findMaximum(Eigen::Index& maxAngleBin, Eigen::Index& maxRadBin) const {
        return data.maxCoeff(&maxAngleBin, &maxRadBin);
    }

    /**
     * @brief Gets a one-bin-wide slice of the Hough space in the angle dimension.
     *
     * This is like taking a single row / column of the Hough space data.
     *
     * @param  angleBin The bin corresponding to the desired slice in the angle dimension.
     * @return          The slice. This is a writable Eigen array block.
     */
    inline DataBlockType getAngularSlice(const Eigen::Index angleBin) { return getAngularSlice(angleBin, 1); }

    /**
     * @brief Const version of getAngularSlice
     */
    inline ConstDataBlockType getAngularSlice(const Eigen::Index angleBin) const {
        return getAngularSlice(angleBin, 1);
    }

    /**
     * @brief Get a block of the matrix in the angular dimension.
     *
     * This version of the function gets a slice that is more than one bin wide. The slice starts
     * at the given bin and has the given width.
     *
     * @param  minAngleBin The first bin of the desired slice.
     * @param  width       The number of bins to include in the slice.
     * @return             A block of the data matrix.
     */
    inline DataBlockType getAngularSlice(const Eigen::Index minAngleBin, const Eigen::Index width) {
        const Eigen::Index minBinClip = std::max(minAngleBin, Eigen::Index{0});
        const Eigen::Index widthClip = std::min(width, numBins - minAngleBin) - (minBinClip - minAngleBin);
        return data.block(minBinClip, 0, widthClip, numBins);
    }

    /**
     * @brief Const version of getAngularSlice
     */
    inline ConstDataBlockType getAngularSlice(const Eigen::Index minAngleBin, const Eigen::Index width) const {
        const Eigen::Index minBinClip = std::max(minAngleBin, Eigen::Index{0});
        const Eigen::Index widthClip = std::min(width, numBins - minAngleBin) - (minBinClip - minAngleBin);
        return data.block(minBinClip, 0, widthClip, numBins);
    }

    /**
     * @brief Gets a one-bin-wide slice of the Hough space in the radius dimension.
     *
     * This is like taking a single row / column of the Hough space data.
     *
     * @param  radiusBin The bin corresponding to the desired slice in the radius dimension.
     * @return           The slice. This is a writable Eigen array block.
     */
    inline DataBlockType getRadialSlice(const Eigen::Index radiusBin) {
        return getRadialSlice(radiusBin, 1);
    }

    /**
     * @brief Const version of getRadialSlice
     */
    inline ConstDataBlockType getRadialSlice(const Eigen::Index radiusBin) const {
        return getRadialSlice(radiusBin, 1);
    }

    /**
     * @brief Get a block of the matrix in the radius dimension.
     *
     * This version of the function gets a slice that is more than one bin wide. The slice starts
     * at the given bin and has the given width.
     *
     * @param  minRadiusBin The first bin of the desired slice.
     * @param  width        The number of bins to include in the slice.
     * @return              A block of the data matrix.
     */
    inline DataBlockType getRadialSlice(const Eigen::Index minRadiusBin, const Eigen::Index width) {
        const Eigen::Index minBinClip = std::max(minRadiusBin, Eigen::Index{0});
        const Eigen::Index widthClip = std::min(width, numBins - minRadiusBin) - (minBinClip - minRadiusBin);
        return data.block(0, minBinClip, numBins, widthClip);
    }

    /**
     * @brief Const version of getRadialSlice
     */
    inline ConstDataBlockType getRadialSlice(const Eigen::Index minRadiusBin, const Eigen::Index width) const {
        const Eigen::Index minBinClip = std::max(minRadiusBin, Eigen::Index{0});
        const Eigen::Index widthClip = std::min(width, numBins - minRadiusBin) - (minBinClip - minRadiusBin);
        return data.block(0, minBinClip, numBins, widthClip);
    }


    /**
     * @brief Find the radius value corresponding to the given bin.
     *
     * @param  bin The bin number
     * @return     The radius corresponding to the lower bound of this bin.
     */
    template <class BinType>
    inline double findRadiusFromBin(const BinType& bin) const {
        static_assert(std::is_arithmetic<BinType>::value, "Bin type must be a scalar value.");
        return findValue(bin, minRadiusValue, maxRadiusValue);
    }

    /**
     * @brief Find the bin that contains the given radius.
     *
     * @param  radius The radius value.
     * @return        The bin number that contains this radius.
     */
    inline Eigen::Index findBinFromRadius(const double radius) const {
        return findBin(radius, minRadiusValue, maxRadiusValue);
    }

    /**
     * @brief Find the angle corresponding to the given bin.
     *
     * @param  bin The bin number.
     * @return     The angle corresponding to the lower bound of this bin.
     */
    template <class BinType>
    inline double findAngleFromBin(const BinType& bin) const {
        static_assert(std::is_arithmetic<BinType>::value, "Bin type must be a scalar value.");
        return findValue(bin, minAngleValue, maxAngleValue);
    }

    /**
     * @brief Find the bin corresponding to the given angle.
     *
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

    //! Get a const reference to the raw data matrix.
    inline const DataArrayType& getData() const { return data; }

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
    template <class BinType>
    inline double findValue(const BinType& bin, const double lowerBound, const double upperBound) const {
        static_assert(std::is_arithmetic<BinType>::value, "Bin type must be a scalar value.");
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
