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

HoughSpace HoughTransform::findHoughSpace(const Eigen::Ref<const Eigen::ArrayXd>& xs,
                                          const Eigen::Ref<const Eigen::ArrayXd>& ys) const {
    if (xs.rows() < rowOffset) {
        throw TooFewPointsException();
    }

    HoughSpace hspace {numBins, maxRadiusValue};

    #pragma omp parallel for
    for (Eigen::Index angleBin = 0; angleBin < hspace.getNumBins(); angleBin++) {
        const double angle = hspace.findAngleFromBin(angleBin);
        const Eigen::ArrayXd radii = radiusFunction(xs, ys, angle);

        for (Eigen::Index radIdx = 0; radIdx < radii.rows(); radIdx++) {
            const double rad = radii(radIdx);
            if (rad >= hspace.getMinRadiusValue() && rad < hspace.getMaxRadiusValue()) {
                const Eigen::Index radBin = hspace.findBinFromRadius(rad);
                hspace.getValueAtBin(angleBin, radBin) += 1;
            }
        }
    }

    return hspace;
}

}
}
