#ifndef ATTPC_CLEANING_CLUSTER_H
#define ATTPC_CLEANING_CLUSTER_H

#include <vector>
#include "attpc/cleaning/eigen_common.h"
#include <pcl/io/io.h>

namespace attpc {
namespace cleaning {

class Cluster {
protected:
    size_t pointIndexCount;
    std::vector<pcl::PointIndicesPtr> clusters;
    mutable Eigen::ArrayXXi relationshipMatrix;

    void calculateRelationshipMatrixIfNecessary() const;

public:
    Cluster() {}

    Cluster(std::vector<pcl::PointIndicesPtr> const& clusters_, size_t pointIndexCount_);

    std::vector<pcl::PointIndicesPtr> const& getClusters() const;

    size_t getPointIndexCount() const;

    int operator-(Cluster const& rhs) const;
};

}
}

#endif //ATTPC_CLEANING_CLUSTER_H
