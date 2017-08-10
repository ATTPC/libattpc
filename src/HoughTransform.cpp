//
// Created by Joshua Bradt on 7/27/17.
//

#include "HoughTransform.h"

namespace attpc {
namespace cleaning {

HoughTransform::HoughTransform(const Eigen::Index numBins_, const double maxRadiusValue_, const Eigen::Index rowOffset_)
: numBins(numBins_)
, rowOffset(rowOffset_)
, maxRadiusValue(maxRadiusValue_)
{}

HoughSpace HoughTransform::findHoughSpace(const Eigen::ArrayXXd& data) const {
    HoughSpace hspace {numBins, maxRadiusValue};

    for (Eigen::Index angleBin = 0; angleBin < hspace.getNumBins(); angleBin++) {
        double angle = hspace.findAngleFromBin(angleBin);
        // Precompute sin and cos here so they aren't done nrows times for each angle
        double cosAngle = std::cos(angle);
        double sinAngle = std::sin(angle);

        for (Eigen::Index xyz_idx = rowOffset; xyz_idx < data.rows(); xyz_idx++) {
            double rad = radiusFunction(data, xyz_idx, cosAngle, sinAngle);
            if (rad >= hspace.getMinRadiusValue() && rad < hspace.getMaxRadiusValue()) {
                // Find and increment histogram/accumulator bin corresponding to rad
                Eigen::Index radBin = hspace.findBinFromRadius(rad);
                hspace.getValueAtBin(angleBin, radBin) += 1;  // TODO: Check if this is actually thread-safe...
            }
        }
    }

    return hspace;
}

}
}
