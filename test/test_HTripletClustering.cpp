//
// Created by Joshua Bradt on 7/27/17.
//

#include "catch.hpp"
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "HTripletClustering.h"

static pcl::PointCloud<pcl::PointXYZI>::Ptr readTestData(const std::string& path) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZI>};
    pcl::io::loadPCDFile(path, *cloud);
    return cloud;
}

static attpc::cleaning::HTripletClusteringConfig makeClusteringConfig() {
    attpc::cleaning::HTripletClusteringConfig config {};

    config.genTripletsNnCandidates = 14;
    config.genTripletsNBest = 2;
    config.cleanupMinTriplets = 20;
    config.smoothRadius = 0.818581f;
    config.genTripletsMaxError = 0.0103171f;
    config.bestClusterDistanceDelta = 2.91713f;
    config.smoothUsingMedian = false;
    config.clusterMetric = attpc::cleaning::singleLinkClusterMetric;
    config.tripletMetric = attpc::cleaning::spiralTripletMetric;

    return config;
}

// TODO: Actually test something here
TEST_CASE("Cloud can be smoothed", "[clustering]") {
    auto testCloud = readTestData("data/data13.pcd");

    auto config = makeClusteringConfig();
    attpc::cleaning::HTripletClustering clusterer {config};

    auto smoothed = clusterer.smoothCloud(testCloud);
}