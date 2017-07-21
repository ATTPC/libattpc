#include <chrono>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include "hc.h"
#include "mst.h"
#include "smoothenCloud.h"

struct hc_params
{
    float cloudScaleModifier;
    size_t genTripletsNnKandidates;
    size_t genTripletsNBest;
    size_t cleanupMinTriplets;
    float smoothRadius;
    float genTripletsMaxError;
    float bestClusterDistanceDelta;
    float _padding;
};

static float calculateCloudScale(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    float totalDistance = 0.0f;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<int> nnIndices;
    nnIndices.reserve(2);
    std::vector<float> nnbSquaredDistances;
    nnbSquaredDistances.reserve(2);

    for (size_t pointIndex = 0; pointIndex < cloud->size(); ++pointIndex)
    {
        int const nnFound = kdtree.nearestKSearch(*cloud, (int)pointIndex, 2, nnIndices, nnbSquaredDistances);

        if (nnFound == 2)
        {
            pcl::PointXYZI const &pointA = (*cloud)[pointIndex];
            pcl::PointXYZI const &pointB = (*cloud)[nnIndices[1]];

            float const distance = std::sqrt(
                (pointA.x - pointB.x) * (pointA.x - pointB.x) +
                (pointA.y - pointB.y) * (pointA.y - pointB.y) +
                (pointA.z - pointB.z) * (pointA.z - pointB.z)
            );

            // std::cout << "TOTDIS: " << distance << "(" << pointIndex << ", " << nnIndices[1] << ")" << std::endl;

            totalDistance += distance;
        }
    }

    // std::cout << "TOT: " << totalDistance << std::endl;
    return totalDistance / (float)cloud->size();
}

Cluster useHc(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<hc::triplet> triplets, float scale, float bestClusterDistanceDelta, size_t cleanupMinTriplets)
{
    hc::cluster_history result = hc::calculateHc(cloud, triplets, hc::singleLinkClusterMetric, [&] (hc::triplet const &lhs, hc::triplet const &rhs, pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
    {
        float const perpendicularDistanceA = (rhs.center - (lhs.center + lhs.direction.dot(rhs.center - lhs.center) * lhs.direction)).squaredNorm();
        float const perpendicularDistanceB = (lhs.center - (rhs.center + rhs.direction.dot(lhs.center - rhs.center) * rhs.direction)).squaredNorm();

        float const angle = std::abs(std::tan(2.0f * std::acos(lhs.direction.dot(rhs.direction))));

        //std::cout << "HI" << perpendicularDistanceA << ", " << perpendicularDistanceB << ", " << std::max(perpendicularDistanceA, perpendicularDistanceB) << ',' << std::pow(2.0f, 1.0f + 12.0f * angle) << std::endl;

        // squared distances!
        //return std::max(perpendicularDistanceA, perpendicularDistanceB) + std::pow(2.0f, 1.0f + 12.0f * angle);
        return std::sqrt(std::max(perpendicularDistanceA, perpendicularDistanceB)) + scale * angle;
    });

    //return hc::toCluster(triplets, result.history[220], cloud->size());

    hc::cluster_group const &clusterGroup = hc::getBestClusterGroup(result, bestClusterDistanceDelta);
    hc::cluster_group const &cleanedUpClusterGroup = hc::cleanupClusterGroup(clusterGroup, cleanupMinTriplets);

    return hc::toCluster(triplets, cleanedUpClusterGroup, cloud->size());
}

void colorByIndex(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb)
{
    cloud_rgb->clear();
    copyPointCloud(*cloud, *cloud_rgb);

    size_t index = 0;
    for(auto it = cloud_rgb->begin(); it != cloud_rgb->end(); ++it)
    {
        it->r = (uint8_t)(256 * (index % 8) / 8);
        it->g = it->r / 2;
        it->b = it->r;
        ++index;
    }
}

void colorByIntensity(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb, float minIntensity = 0.0f, float maxIntensity = 4000.0f)
{
    cloud_rgb->clear();
    copyPointCloud(*cloud, *cloud_rgb);

    float size = maxIntensity - minIntensity;

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        auto &point = (*cloud_rgb)[i];

        point.r = (uint8_t)(256 * (((*cloud)[i].intensity - minIntensity) / size));
        point.g = point.r / 2;
        point.b = point.r;
    }
}

void colorByCluster(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, Cluster const &cluster, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb)
{
    cloud_rgb->clear();
    copyPointCloud(*cloud, *cloud_rgb);

    // default color: red
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        auto &point = (*cloud_rgb)[i];

        point.r = 255;
        point.g = 0;
        point.b = 0;
    }

    size_t clusterIndex = 0;
    for (pcl::PointIndicesPtr const &pointIndices : cluster.getClusters())
    {
        double const r = (double)((clusterIndex * 23) % 19) / 18.0;
        double const g = (double)((clusterIndex * 23) % 7) / 6.0;
        double const b = (double)((clusterIndex * 23) % 3) / 2.0;

        for (int index : pointIndices->indices)
        {
            auto &point = (*cloud_rgb)[index];

            point.r = (uint8_t)(r * 255);
            point.g = (uint8_t)(g * 255);
            point.b = (uint8_t)(b * 255);
        }

        ++clusterIndex;
    }
}

int main()
{

    std::ofstream hcParamsResults("hcParamsResults.txt", std::ofstream::app);

    hc_params bestParams;

    // all (7697)
    bestParams.cloudScaleModifier = 0.281718f;
    bestParams.smoothRadius = 0.818581f;
    bestParams.genTripletsNnKandidates = 14;
    bestParams.genTripletsNBest = 2;
    bestParams.genTripletsMaxError = 0.0103171f;
    bestParams.bestClusterDistanceDelta = 2.91713f;
    bestParams.cleanupMinTriplets = 20;



    std::vector<std::string> filenames;

    filenames.push_back("data03-spiral.pcd");
    filenames.push_back("data13.pcd");
    filenames.push_back("data40.pcd");
    filenames.push_back("data41.pcd");
    filenames.push_back("data44-spiral.pcd");
    filenames.push_back("data49.pcd");
    filenames.push_back("data62.pcd");
    filenames.push_back("data99.pcd");
    filenames.push_back("data190-spiral.pcd");

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;

    for (std::string const &filename : filenames)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::io::loadPCDFile("data/nscl-pcd-ex/" + filename, *cloud);
        //pcl::io::loadPCDFile("data/nscl-pcd-ex-new/" + filename, *cloud);

        clouds.push_back(cloud);
    }

    while (true)
    {
        std::vector<hc_params> hcParams;

        hcParams.push_back(bestParams);

        for (int paramIndex = 0; paramIndex < (int)hcParams.size(); ++paramIndex)
        {

            hc_params const &currentHcParams = hcParams[paramIndex];

            for (size_t fileIndex = 0; fileIndex < filenames.size(); ++fileIndex)
            {

                    std::string currentFile = filenames[fileIndex];
                    pcl::PointCloud<pcl::PointXYZI>::Ptr const &cloud_xyzti = clouds[fileIndex];
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());

                    copyPointCloud(*cloud_xyzti, *cloud_xyz);

                    if (cloud_xyz->size() == 0)
                        std::cout << "empty cloud: " << currentFile << '!' << std::endl;
                    else {
                        //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
                        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliner(new pcl::PointCloud<pcl::PointXYZ>());

                        /*
                        // outliner removal
                        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorfilter(true);
                        sorfilter.setInputCloud(cloud_xyz);
                        sorfilter.setMeanK(12);
                        sorfilter.setStddevMulThresh(2.0);
                        sorfilter.filter(*cloud_filtered);

                        // get removed indices
                        pcl::IndicesConstPtr indices_rem = sorfilter.getRemovedIndices();
                        pcl::ExtractIndices<pcl::PointXYZ> extract;
                        extract.setInputCloud(cloud_xyz);
                        extract.setIndices(indices_rem);
                        extract.setNegative(false);
                        extract.filter(*cloud_outliner);
                        */

                        // calculate cloud-scale
                        float const cloudScale = calculateCloudScale(cloud_xyzti);
                        // std::cout << "XX cloudScale: " << cloudScale << std::endl;

                        // smoothen cloud
                        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzti_smooth(new pcl::PointCloud<pcl::PointXYZI>());

                        //cloud_smooth = smoothenCloud(cloud_filtered, 12); // k nearest neighbour
                        cloud_xyzti_smooth = smoothenCloud(cloud_xyzti, cloudScale * currentHcParams.smoothRadius); // radius

                        // calculate cluster
                        Cluster cluster;
                        std::vector<hc::triplet> triplets;

                        {
                            // calculate
                            triplets = hc::generateTriplets(cloud_xyzti_smooth, currentHcParams.genTripletsNnKandidates, currentHcParams.genTripletsNBest, currentHcParams.genTripletsMaxError);
                            cluster = useHc(cloud_xyzti_smooth, triplets, cloudScale * currentHcParams.cloudScaleModifier, currentHcParams.bestClusterDistanceDelta, currentHcParams.cleanupMinTriplets);
                        }

                    }
            }

        }

    }

    return 0;
}
