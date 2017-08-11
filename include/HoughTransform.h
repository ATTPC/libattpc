//
// Created by Joshua Bradt on 7/27/17.
//

#ifndef ATTPC_CLEANING_HOUGHTRANSFORM_H
#define ATTPC_CLEANING_HOUGHTRANSFORM_H

#include <cstddef>
#include <Eigen/Core>
#include "HoughSpace.h"

namespace attpc {
namespace cleaning {

/**
 * Base class implementing the Hough transform.
 *
 * This must be used through a child class that overrides the radiusFunction.
 */
class HoughTransform {
public:
    /**
     * Constructor
     *
     * @param numBins_        The number of bins to use in the Hough space. The Hough space matrix will be square.
     * @param maxRadiusValue_ The scale of the Hough space.
     * @param rowOffset_      The first data point to use when calculating the Hough space. This should typically
     *                        be zero, which is the default value.
     */
    HoughTransform(const Eigen::Index numBins_, const double maxRadiusValue_, const Eigen::Index rowOffset_ = 0);

    virtual ~HoughTransform() = default;

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
    HoughSpace findHoughSpace(const Eigen::ArrayXXd& data) const;

    //! Get the number of bins.
    Eigen::Index getNumBins() const { return numBins; }

    //! Set the number of bins.
    void setNumBins(const Eigen::Index newNumBins) { numBins = newNumBins; }

    //! Get the first row of the data that will be processed.
    Eigen::Index getRowOffset() const { return rowOffset; }

    //! Set the first row of the data that will be processed.
    void setRowOffset(const Eigen::Index newRowOffset) { rowOffset = newRowOffset; }

    //! Get the radius corresponding to the upper bound of the last bin.
    double getMaxRadiusValue() const { return maxRadiusValue; }

    //! Set the radius corresponding to the upper bound of the last bin.
    void setMaxRadiusValue(const double newMaxRadiusValue) { maxRadiusValue = newMaxRadiusValue; }

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
    //! The number of bins to use for the Hough space in both dimensions.
    Eigen::Index numBins;

    //! The index of the first data point to consider.
    Eigen::Index rowOffset;

    //! The upper bound of the Hough space in the radius dimension.
    double maxRadiusValue;
};

}
}


#endif //ATTPC_CLEANING_HOUGHTRANSFORM_H
