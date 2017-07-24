//
// Created by Joshua Bradt on 7/24/17.
//

#include "metrics/CompleteLinkClusterMetric.h"

float hc::CompleteLinkClusterMetric::operator()(const cluster& lhs, const cluster& rhs, const Eigen::MatrixXf& d,
                                                pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
{
    float result = 0.0f;

    for (size_t const &a : lhs)
    {
        for (size_t const &b : rhs)
        {
            float distance = d(a, b);

            if (distance > result)
                result = distance;
        }
    }

    return result;
}
