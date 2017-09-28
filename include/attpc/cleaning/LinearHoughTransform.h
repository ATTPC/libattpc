#ifndef ATTPC_CLEANING_LINEARHOUGHTRANSFORM_H
#define ATTPC_CLEANING_LINEARHOUGHTRANSFORM_H

#include "attpc/cleaning/eigen_common.h"
#include "attpc/cleaning/HoughTransform.h"

namespace attpc {
namespace cleaning {

/**
 * @brief The Hough transform for lines.
 */
class LinearHoughTransform : public HoughTransform {
public:
    /**
     * @brief Constructor
     *
     * @param numBins_        The number of bins to use in each dimension of the Hough space.
     * @param maxRadiusValue_ The radius corresponding to the largest radial bin in the Hough space.
     */
    LinearHoughTransform(const Eigen::Index numBins_, const double maxRadiusValue_);

protected:
    /**
     * @brief The radius function for the linear Hough transform.
     *
     * @param  xs   The x coordinates of the data.
     * @param  ys   The y coordinates of the data.
     * @param  angle The angle value to use in the computation.
     * @return       An array of radius values computed for the data points. The length of this array will be
     *               the same as the number of rows in the data.
     */
     Eigen::ArrayXd radiusFunction(const Eigen::Ref<const Eigen::ArrayXd>& xs,
                                   const Eigen::Ref<const Eigen::ArrayXd>& ys,
                                   const double angle) const override;
};

}
}

#endif //ATTPC_CLEANING_LINEARHOUGHTRANSFORM_H
