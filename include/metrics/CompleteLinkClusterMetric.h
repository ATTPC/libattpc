//
// Created by Joshua Bradt on 7/24/17.
//

#ifndef CLOUD_VIEWER_COMPLETELINKCLUSTERMETRIC_H
#define CLOUD_VIEWER_COMPLETELINKCLUSTERMETRIC_H

#include "metrics/ClusterMetric.h"

namespace hc
{
    class CompleteLinkClusterMetric : public ClusterMetric
    {
    public:
        float operator()(cluster const& lhs, cluster const& rhs, Eigen::MatrixXf const& d,
                         pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) override;
    };
}


#endif //CLOUD_VIEWER_COMPLETELINKCLUSTERMETRIC_H
