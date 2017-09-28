//
// Created by Joshua Bradt on 7/25/17.
//

#include "attpc/cleaning/metrics.h"


namespace attpc {
namespace cleaning {

float singleLinkClusterMetric(const cluster& lhs, const cluster& rhs, Eigen::MatrixXf const& d) {
    float result = std::numeric_limits<float>::infinity();

    for (size_t const& a : lhs) {
        for (size_t const& b : rhs) {
            float distance = d(a, b);

            if (distance < result)
                result = distance;
        }
    }

    return result;
}

float completeLinkClusterMetric(const cluster& lhs, const cluster& rhs, const Eigen::MatrixXf& d) {
    float result = 0.0f;

    for (size_t const& a : lhs) {
        for (size_t const& b : rhs) {
            float distance = d(a, b);

            if (distance > result)
                result = distance;
        }
    }

    return result;
}

float spiralTripletMetric(const Triplet& lhs, const Triplet& rhs) {
    float const perpendicularDistanceA = (rhs.center - (lhs.center + lhs.direction.dot(rhs.center - lhs.center) *
                                                                     lhs.direction)).squaredNorm();
    float const perpendicularDistanceB = (lhs.center - (rhs.center + rhs.direction.dot(lhs.center - rhs.center) *
                                                                     rhs.direction)).squaredNorm();

    float const angle = 1.0f - std::abs(lhs.direction.dot(rhs.direction));

    // squared distances!
    return std::max(perpendicularDistanceA, perpendicularDistanceB) + std::pow(2.0f, 1.0f + 12.0f * angle);
}

}
}
