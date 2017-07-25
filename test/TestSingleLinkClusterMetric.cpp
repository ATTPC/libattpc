//
// Created by Joshua Bradt on 7/25/17.
//

#include "catch.hpp"
#include <Eigen/Core>
#include <pcl/common/common.h>
#include <HTripletClustering.h>
#include "metrics/SingleLinkClusterMetric.h"

namespace atc = attpc::clustering;
using atc::HTripletClustering::point_type;
using atc::HTripletClustering::cloud_type;

class MetricTestData {
public:
    MetricTestData(const int numPts_)
    : numPts(numPts_)
    {
        for (int ptIdx = 0; ptIdx < numPts; ++ptIdx) {
            cloud.push_back({ptIdx, ptIdx, ptIdx, ptIdx});
        }


    }

public:
    const int numPts;
    cloud_type cloud;
    Eigen::ArrayXXf distanceMatrix;
    atc::cluster clusterA;
    atc::cluster clusterB;
};

TEST_CASE("SingleLinkClusterMetric works", "[metrics]") {

}