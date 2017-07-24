//
// Created by Joshua Bradt on 7/24/17.
//

#ifndef CLOUD_VIEWER_TRIPLETMETRIC_H
#define CLOUD_VIEWER_TRIPLETMETRIC_H

#include <pcl/common/common.h>
#include "Triplet.h"

namespace hc
{
    // typedef std::function<float(triplet const &lhs, triplet const &rhs, pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)> TripletMetric;
    class TripletMetric
    {
    public:
        virtual float operator()(Triplet const &lhs, Triplet const &rhs, pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) = 0;
    };
}

#endif //CLOUD_VIEWER_TRIPLETMETRIC_H
