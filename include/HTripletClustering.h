#ifndef HC_CLUSTERING_H
#define HC_CLUSTERING_H

#include <memory>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "Triplet.h"
#include "metrics/ClusterMetric.h"
#include "metrics/SingleLinkClusterMetric.h"
#include "metrics/TripletMetric.h"
#include "metrics/SpiralTripletMetric.h"

namespace hc {
    typedef std::vector<size_t> cluster;

    struct cluster_group
    {
        std::vector<cluster> clusters;
        float bestClusterDistance;
    };

    struct cluster_history
    {
        std::vector<Triplet> triplets;
        std::vector<cluster_group> history;
    };

    class HTripletClustering {
    public:
        using point_type = pcl::PointXYZI;
        using cloud_type = pcl::PointCloud<point_type>;

        HTripletClustering();

        void generateSmoothedCloud();
        void generateTriplets();
        cluster_history calculateHc();


    private:
        float cloudScaleModifier;
        size_t genTripletsNnCandidates;
        size_t genTripletsNBest;
        size_t cleanupMinTriplets;
        float smoothRadius;
        float genTripletsMaxError;
        float bestClusterDistanceDelta;
        bool smoothUsingMedian;

        cloud_type::ConstPtr xyziCloud;
        cloud_type::Ptr smoothCloud;
        std::vector<Triplet> triplets;

        std::unique_ptr<ClusterMetric> clusterMetric;
        std::unique_ptr<TripletMetric> tripletMetric;
    };
}

#endif
