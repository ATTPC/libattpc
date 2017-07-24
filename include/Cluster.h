#ifndef ATTPC_CLUSTERING_CLUSTER_H
#define ATTPC_CLUSTERING_CLUSTER_H

#include <vector>
#include <pcl/io/io.h>

namespace attpc {
namespace clustering {

class Cluster {
protected:
    size_t pointIndexCount;
    std::vector<pcl::PointIndicesPtr> clusters;
    mutable Eigen::ArrayXXi relationshipMatrix;

    void calculateRelationshipMatrixIfNecessary() const;

public:
    Cluster() {};

    Cluster(std::vector<pcl::PointIndicesPtr> const& clusters, size_t pointIndexCount);

    std::vector<pcl::PointIndicesPtr> const& getClusters() const;

    size_t getPointIndexCount() const;

    int operator-(Cluster const& rhs) const;
};

}
}

#endif //ATTPC_CLUSTERING_CLUSTER_H
