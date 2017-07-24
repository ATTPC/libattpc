#ifndef HC_CLUSTERING_H
#define HC_CLUSTERING_H

#include <memory>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "Triplet.h"

namespace hc {
    class HTripletClustering {
    public:
        using point_type = pcl::PointXYZI;
        using cloud_type = pcl::PointCloud<point_type>;

        HTripletClustering();

        void generateSmoothedCloud();
        void generateTriplets();


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
    };
}

#endif
