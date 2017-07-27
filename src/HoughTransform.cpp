//
// Created by Joshua Bradt on 7/27/17.
//

#include "HoughTransform.h"

namespace attpc {
namespace cleaning {

HoughTransform::HoughTransform(const int numBins_, const int maxRadiusValue_)
: numBins(numBins_)
, maxRadiusValue(maxRadiusValue_)
{}

Eigen::ArrayXXf HoughTransform::findHoughSpace(const Eigen::ArrayXXd& data) const {
    const double thstep = M_PI / numBins;  // Size of theta (angle) bins
    const double minRadiusValue = -maxRadiusValue;

    Eigen::ArrayXXd result = Eigen::ArrayXXd::Zero(data.rows(), data.cols());

    #pragma omp parallel for
    for (int theta_idx = 0; theta_idx < numBins; theta_idx++) {
        double theta = theta_idx * thstep;
        // Precompute sin and cos here so they aren't done nrows times for each theta
        double costh = cos(theta);
        double sinth = sin(theta);

        for (int xyz_idx = rowOffset; xyz_idx < data.rows(); xyz_idx++) {
            double rad = radiusFunction(data, xyz_idx, costh, sinth);
            if (rad >= minRadiusValue && rad < maxRadiusValue) {
                // Find and increment histogram/accumulator bin corresponding to rad
                size_t radbin = (size_t) floor((rad + maxRadiusValue) * numBins / (2 * maxRadiusValue));
                result(theta_idx, radbin) += 1;  // TODO: Check if this is actually thread-safe...
            }
        }
    }
}

double HoughTransform::findRadiusFromBin(const Eigen::Index bin) const {
    return bin * 2 * maxRadiusValue / numBins - maxRadiusValue;
}

double HoughTransform::findAngleFromBin(const Eigen::Index bin) const {
    return bin * M_PI / numBins;
}

}
}