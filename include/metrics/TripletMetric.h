//
// Created by Joshua Bradt on 7/24/17.
//

#ifndef ATTPC_CLUSTERING_TRIPLETMETRIC_H
#define ATTPC_CLUSTERING_TRIPLETMETRIC_H

#include <pcl/common/common.h>
#include "Triplet.h"

namespace attpc {
namespace clustering {

// typedef std::function<float(triplet const &lhs, triplet const &rhs, pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)> TripletMetric;
class TripletMetric {
public:
    virtual float
    operator()(Triplet const& lhs, Triplet const& rhs, pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud) = 0;
};

}
}

#endif //ATTPC_CLUSTERING_TRIPLETMETRIC_H
