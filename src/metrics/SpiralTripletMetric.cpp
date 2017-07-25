//
// Created by Joshua Bradt on 7/24/17.
//

#include "metrics/SpiralTripletMetric.h"

namespace attpc {
namespace clustering {

float SpiralTripletMetric::operator()(const Triplet& lhs, const Triplet& rhs) {
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