//
// Created by Joshua Bradt on 7/24/17.
//

#ifndef CLOUD_VIEWER_SPIRALTRIPLETMETRIC_H
#define CLOUD_VIEWER_SPIRALTRIPLETMETRIC_H

#include <pcl/common/common.h>
#include <cmath>
#include <algorithm>
#include "metrics/TripletMetric.h"

namespace hc {
    class SpiralTripletMetric : public TripletMetric
    {
    public:
        float
        operator()(Triplet const& lhs, Triplet const& rhs, pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) override;
    };
}


#endif //CLOUD_VIEWER_SPIRALTRIPLETMETRIC_H
