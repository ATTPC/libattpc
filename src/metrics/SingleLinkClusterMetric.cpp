//
// Created by Joshua Bradt on 7/24/17.
//

#include "metrics/SingleLinkClusterMetric.h"

float hc::SingleLinkClusterMetric::operator()(const hc::cluster& lhs, const hc::cluster& rhs, Eigen::MatrixXf const& d,
                                              pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
    float result = std::numeric_limits<float>::infinity();

    for (size_t const &a : lhs)
    {
        for (size_t const &b : rhs)
        {
            float distance = d(a, b);

            if (distance < result)
                result = distance;
        }
    }

    return result;
}
