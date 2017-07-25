//
// Created by Joshua Bradt on 7/24/17.
//

#ifndef ATTPC_CLUSTERING_COMPLETELINKCLUSTERMETRIC_H
#define ATTPC_CLUSTERING_COMPLETELINKCLUSTERMETRIC_H

#include "metrics/ClusterMetric.h"

namespace attpc {
namespace clustering {

class CompleteLinkClusterMetric : public ClusterMetric {
public:
    /**
     * A metric for comparing two clusters using a complete-linkage criterion.
     * @param lhs The first cluster to compare.
     * @param rhs The second cluster to compare.
     * @param d The distance matrix between points in the data.
     * @return The distance between the two clusters.
     */
    float operator()(cluster const& lhs, cluster const& rhs, Eigen::MatrixXf const& d) override;
};

}
}


#endif //ATTPC_CLUSTERING_COMPLETELINKCLUSTERMETRIC_H
