#ifndef ATTPC_CLEANING_HTRIPLETCLUSTERING_H
#define ATTPC_CLEANING_HTRIPLETCLUSTERING_H

#include "attpc/cleaning/eigen_common.h"
#include <memory>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "attpc/cleaning/Triplet.h"
#include "attpc/cleaning/Cluster.h"
#include "attpc/cleaning/metrics.h"
#include "attpc/cleaning/utilities.h"

namespace attpc {
namespace cleaning {

struct cluster_group {
    std::vector<cluster> clusters;
    float bestClusterDistance;
};

struct cluster_history {
    std::vector<Triplet> triplets;
    std::vector<cluster_group> history;
};

struct HTripletClusteringConfig {
    size_t genTripletsNnCandidates;
    size_t genTripletsNBest;
    size_t cleanupMinTriplets;
    float smoothRadius;
    float genTripletsMaxError;
    float bestClusterDistanceDelta;
    bool smoothUsingMedian;

    ClusterMetric clusterMetric;
    TripletMetric tripletMetric;
};

class HTripletClustering {
public:
    using point_type = pcl::PointXYZI;
    using cloud_type = pcl::PointCloud<point_type>;

    HTripletClustering();

    HTripletClustering(const HTripletClusteringConfig& config);

    void setParamsFromConfig(const HTripletClusteringConfig& config);

    cloud_type smoothCloud(cloud_type::ConstPtr cloud) const;

    std::vector<Triplet> generateTriplets(cloud_type::ConstPtr cloud) const;

    cluster_history calculateHc(cloud_type::ConstPtr cloud, const std::vector<Triplet>& triplets) const;

    cluster_group findBestClusterGroup(const cluster_history& history) const;

    cluster_group cleanupClusterGroup(cluster_group const& clusterGroup) const;

    Cluster
    makeCluster(const std::vector<Triplet>& triplets, const cluster_group& clusterGroup, size_t pointIndexCount) const;

private:
    size_t genTripletsNnCandidates;
    size_t genTripletsNBest;
    size_t cleanupMinTriplets;
    float smoothRadius;
    float genTripletsMaxError;
    float bestClusterDistanceDelta;
    bool smoothUsingMedian;

    ClusterMetric clusterMetric;
    TripletMetric tripletMetric;
};

}
}

#endif //ATTPC_CLEANING_HTRIPLETCLUSTERING_H
