//
// Created by Joshua Bradt on 7/24/17.
//

#ifndef ATTPC_CLUSTERING_SINGLELINKCLUSTERMETRIC_H
#define ATTPC_CLUSTERING_SINGLELINKCLUSTERMETRIC_H

#include "metrics/ClusterMetric.h"

namespace attpc {
namespace clustering {

class SingleLinkClusterMetric : public ClusterMetric {
public:
    float operator()(cluster const& lhs, cluster const& rhs, Eigen::MatrixXf const& d,
                     pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) override;
};

}
}

#endif //ATTPC_CLUSTERING_SINGLELINKCLUSTERMETRIC_H
