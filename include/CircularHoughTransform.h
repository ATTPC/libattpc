//
// Created by Joshua Bradt on 7/27/17.
//

#ifndef ATTPC_CLEANING_CIRCULARHOUGHTRANSFORM_H
#define ATTPC_CLEANING_CIRCULARHOUGHTRANSFORM_H

#include "HoughTransform.h"

namespace attpc {
namespace cleaning {

/**
 * @brief The Hough transform for circles.
 *
 * This class can be used to find the center of a spiral-shaped track.
 *
 */
class CircularHoughTransform : public HoughTransform {
public:
    /**
     * @brief Constructor
     *
     * @param numBins_        The number of bins to use in each dimension in the Hough space.
     * @param maxRadiusValue_ The radius corresponding to the largest radial bin in the Hough space.
     * @param rowOffset_      Separation between points that will be considered when finding the center. For example,
     *                        if the rowOffset_ is 5, points x[i] and x[i-5] will be used in the computation for
     *                        the ith point.
     */
    CircularHoughTransform(const Eigen::Index numBins_, const double maxRadiusValue_, const Eigen::Index rowOffset_ = 5);

    /**
     * @brief Find the center of a spiral.
     *
     * @param  xs   The x coordinates of the data.
     * @param  ys   The y coordinates of the data.
     * @return      The center (x, y) of the spiral as a 2D vector.
     * @throws HoughTransform::TooFewPointsException If the number of rows in the data is less than rowOffset.
     */
    Eigen::Vector2d findCenter(const Eigen::Ref<const Eigen::ArrayXd>& xs,
                               const Eigen::Ref<const Eigen::ArrayXd>& ys) const;

protected:
    /**
     * @brief The radius function for the circular transform.
     *
     * @param  xs   The x coordinates of the data.
     * @param  ys   The y coordinates of the data.
     * @param  angle The angle in the Hough space that should be used in the calculation.
     * @return       An array of radius values computed for the data points. The length of this array will be
     *               less than the length of the data by whatever the value of rowOffset is.
     */
     Eigen::ArrayXd radiusFunction(const Eigen::Ref<const Eigen::ArrayXd>& xs,
                                   const Eigen::Ref<const Eigen::ArrayXd>& ys,
                                   const double angle) const override;
};

}
}

#endif //ATTPC_CLEANING_CIRCULARHOUGHTRANSFORM_H
