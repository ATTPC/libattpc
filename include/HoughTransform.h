//
// Created by Joshua Bradt on 7/27/17.
//

#ifndef ATTPC_CLEANING_HOUGHTRANSFORM_H
#define ATTPC_CLEANING_HOUGHTRANSFORM_H

#include <cstddef>
#include <Eigen/Core>

namespace attpc {
namespace cleaning {

/**
 * Base class implementing the Hough transform.
 *
 * This must be used through a child class that overrides the radiusFunction.
 *
 * @param numBins_        The number of bins to use in the Hough space. The Hough space matrix will be square.
 * @param maxRadiusValue_ The scale of the Hough space.
 * @param rowOffset_      The first data point to use when calculating the Hough space. This should typically
 *                        be zero, which is the default value.
 */
class HoughTransform {
public:
    HoughTransform(const int numBins_, const int maxRadiusValue_, const int rowOffset_ = 0);

    /**
     * Apply the Hough transform to the given dataset.
     *
     * The data should be a matrix with rows corresponding to individual data points. The first
     * column should contain the x values, and the second column should contain the y values.
     *
     * The Hough space is returned as a 2D array with the first index (rows) corresponding to angles
     * and the second index (columns) corresponding to radii. The bounds of the Hough space can be found
     * using the member functions defined in this class.
     *
     * @param  data The data to transform.
     * @return      The Hough space.
     */
    Eigen::ArrayXXd findHoughSpace(const Eigen::ArrayXXd& data) const;

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
    int getNumBins() const { return numBins; }

    //! Get the first row of the data that will be processed.
    int getRowOffset() const { return rowOffset; }

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

protected:
    /**
     * The radius as a function of angle, x, and y.
     *
     * This represents the core of the Hough transform, and it is the mapping between the data space
     * and the Hough space. It must be overridden in a child class to implement a specific form of the
     * Hough transform, like a linear transform or a transform for finding circles.
     *
     * The function should take the full data set, an index pointing to the current row, and the sine
     * and cosine of the current angle. The sine and cosine are pre-computed to improve performance.
     *
     * @param  data   The data, in the same format as used in findHoughSpace.
     * @param  rowIdx The index of the current row.
     * @param  costh  The cosine of the current angle.
     * @param  sinth  The sine of the current angle.
     * @return        The radius value corresponding to the current data point and angle.
     */
    virtual double radiusFunction(const Eigen::ArrayXXd& data, const Eigen::Index rowIdx,
                                  const double costh, const double sinth) const = 0;

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

    //! The number of bins to use for the Hough space in both dimensions.
    int numBins;

    //! The index of the first data point to consider.
    int rowOffset;

    //! The lower bound of the Hough space in the radius dimension.
    double minRadiusValue;

    //! The upper bound of the Hough space in the radius dimension.
    double maxRadiusValue;

    //! The lower bound of the Hough space in the angle dimension.
    double minAngleValue;

    //! The upper bound of the Hough space in the angle dimension.
    double maxAngleValue;
};

}
}


#endif //ATTPC_CLEANING_HOUGHTRANSFORM_H
