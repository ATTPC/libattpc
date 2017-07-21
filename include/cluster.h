#ifndef HC_CLUSTER_H
#define HC_CLUSTER_H

#include <vector>
#include <pcl/io/io.h>

namespace hc {

    class Cluster
    {
    protected:
        size_t pointIndexCount;
        std::vector<pcl::PointIndicesPtr> clusters;
        mutable Eigen::ArrayXXi relationshipMatrix;

        void calculateRelationshipMatrixIfNecessary() const;

    public:
        Cluster() {};
        Cluster(std::vector<pcl::PointIndicesPtr> const &clusters, size_t pointIndexCount);

        std::vector<pcl::PointIndicesPtr> const &getClusters() const;
        size_t getPointIndexCount() const;

        int operator-(Cluster const &rhs) const;
    };

}

#endif
