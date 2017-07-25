#ifndef ATTPC_CLUSTERING_HTRIPLETCLUSTERING_H
#define ATTPC_CLUSTERING_HTRIPLETCLUSTERING_H

#include <memory>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "Triplet.h"
#include "Cluster.h"
#include "metrics/ClusterMetric.h"
#include "metrics/SingleLinkClusterMetric.h"
#include "metrics/TripletMetric.h"
#include "metrics/SpiralTripletMetric.h"
#include "utilities.h"

namespace attpc {
namespace clustering {

typedef std::vector<size_t> cluster;

struct cluster_group {
    std::vector<cluster> clusters;
    float bestClusterDistance;
};

struct cluster_history {
    std::vector<Triplet> triplets;
    std::vector<cluster_group> history;
};

class HTripletClustering {
public:
    using point_type = pcl::PointXYZI;
    using cloud_type = pcl::PointCloud<point_type>;

    HTripletClustering();

    cloud_type smoothCloud(cloud_type::ConstPtr cloud) const;

    std::vector<Triplet> generateTriplets(cloud_type::ConstPtr cloud) const;

    cluster_history calculateHc(cloud_type::ConstPtr cloud, const std::vector<Triplet>& triplets) const;

    cluster_group findBestClusterGroup(const cluster_history& history) const;

    cluster_group cleanupClusterGroup(cluster_group const& clusterGroup) const;

    Cluster
    makeCluster(const std::vector<Triplet>& triplets, const cluster_group& clusterGroup, size_t pointIndexCount) const;

private:
    float cloudScaleModifier;
    size_t genTripletsNnCandidates;
    size_t genTripletsNBest;
    size_t cleanupMinTriplets;
    float smoothRadius;
    float genTripletsMaxError;
    float bestClusterDistanceDelta;
    bool smoothUsingMedian;

    std::unique_ptr<ClusterMetric> clusterMetric;
    std::unique_ptr<TripletMetric> tripletMetric;
};

}
}

#endif //ATTPC_CLUSTERING_HTRIPLETCLUSTERING_H
