//
// Created by Joshua Bradt on 7/24/17.
//

#include "metrics/SingleLinkClusterMetric.h"

namespace attpc {
namespace clustering {

float SingleLinkClusterMetric::operator()(const cluster& lhs, const cluster& rhs, Eigen::MatrixXf const& d) {
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

}
}