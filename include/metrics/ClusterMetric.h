//
// Created by Joshua Bradt on 7/24/17.
//

#ifndef ATTPC_CLUSTERING_CLUSTERMETRIC_H
#define ATTPC_CLUSTERING_CLUSTERMETRIC_H

#include <Eigen/Core>
#include <vector>

namespace attpc {
namespace clustering {

typedef std::vector<size_t> cluster;

class ClusterMetric {
public:
    virtual float operator()(cluster const& lhs, cluster const& rhs, Eigen::MatrixXf const& d) = 0;
};

}
}


#endif //ATTPC_CLUSTERING_CLUSTERMETRIC_H
