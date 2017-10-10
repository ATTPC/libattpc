#ifndef ATTPC_CLEANING_METRICS_H
#define ATTPC_CLEANING_METRICS_H

#include <functional>
#include <vector>
#include <cstddef>
#include "attpc/cleaning/eigen_common.h"
#include "attpc/cleaning/Triplet.h"


namespace attpc {
namespace cleaning {

using cluster = std::vector<size_t>;

using ClusterMetric = std::function<float(const cluster&, const cluster&, const Eigen::MatrixXf&)>;
using TripletMetric = std::function<float(const Triplet&, const Triplet&)>;

/**
 * A metric for comparing two clusters using a single-linkage criterion.
 * @param lhs The first cluster to compare.
 * @param rhs The second cluster to compare.
 * @param d The distance matrix between points in the data.
 * @return The distance between the two clusters.
 */
float singleLinkClusterMetric(const cluster& lhs, const cluster& rhs, const Eigen::MatrixXf& d);

/**
 * A metric for comparing two clusters using a complete-linkage criterion.
 * @param lhs The first cluster to compare.
 * @param rhs The second cluster to compare.
 * @param d The distance matrix between points in the data.
 * @return The distance between the two clusters.
 */
float completeLinkClusterMetric(const cluster& lhs, const cluster& rhs, const Eigen::MatrixXf& d);

float spiralTripletMetric(const Triplet& lhs, const Triplet& rhs);

}
}

#endif //ATTPC_CLEANING_METRICS_H
