//
// Created by Joshua Bradt on 7/24/17.
//

#ifndef CLUSTERMETRIC_H
#define CLUSTERMETRIC_H

#include <pcl/common/common.h>
#include <Eigen/Core>


namespace hc {
    typedef std::vector<size_t> cluster;
    class ClusterMetric
    {
    public:
        virtual float operator()(cluster const &lhs, cluster const &rhs, Eigen::MatrixXf const &d, pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) = 0;
    };
}


#endif //CLUSTERMETRIC_H
