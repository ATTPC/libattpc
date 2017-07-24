//
// Created by Joshua Bradt on 7/24/17.
//

#ifndef ATTPC_CLUSTERING_SPIRALTRIPLETMETRIC_H
#define ATTPC_CLUSTERING_SPIRALTRIPLETMETRIC_H

#include <pcl/common/common.h>
#include <cmath>
#include <algorithm>
#include "metrics/TripletMetric.h"

namespace attpc {
namespace clustering {

class SpiralTripletMetric : public TripletMetric {
public:
    float
    operator()(Triplet const& lhs, Triplet const& rhs, pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) override;
};

}
}

#endif //ATTPC_CLUSTERING_SPIRALTRIPLETMETRIC_H
