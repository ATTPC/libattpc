#define F17_VISUALIZE
//#define F17_OPTIMIZE_PARAMETERS
//#define F17_USE_FILECACHE

#pragma warning(push, 0)
#include <chrono>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#ifdef F17_VISUALIZE
#include <pcl/visualization/cloud_viewer.h>
#endif
#pragma warning(pop)

#include "hc.h"
#include "mst.h"
#include "smoothenCloud.h"

// globals
#ifdef F17_VISUALIZE
static bool wait = true;
static bool reload = false;
#endif

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

#ifdef F17_VISUALIZE
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer*>(viewer_void);

    if(event.keyDown())
    {
        if (event.getKeySym() == "n")
            wait = false;
        else if (event.getKeySym() == "m") {
            reload = true;
            wait = false;
        }
    }
}

void visualizeClusterAsMst(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, Cluster const &cluster)
{
    size_t lineId = 0;
    size_t clusterIndex = 0;

    for (pcl::PointIndicesPtr const &pointIndices : cluster.getClusters())
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(pointIndices);
        extract.setNegative(false);
        extract.filter(*clusterCloud);

        std::vector<mst::state> mstResult = mst::calculateMinimumSpanningTree(clusterCloud);

        if (mstResult.size() < 1)
            continue;

        double const r = (double)((clusterIndex * 23) % 19) / 18.0;
        double const g = (double)((clusterIndex * 23) % 7) / 6.0;
        double const b = (double)((clusterIndex * 23) % 3) / 2.0;

        auto const &lastMstResult = mstResult[mstResult.size() - 1];

        for (auto const &edge : lastMstResult.edges)
        {
            auto const &pointA = (*clusterCloud)[edge.voxelIndexA];
            auto const &pointB = (*clusterCloud)[edge.voxelIndexB];

            std::ostringstream oss;
            oss << "line" << lineId;
            viewer.addLine<pcl::PointXYZI>(pointA, pointB, r, g, b, oss.str());
            ++lineId;
        }

        ++clusterIndex;
    }
}

void visualizeTriplets(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<hc::triplet> const &triplets)
{
    size_t lineId = 0;
    size_t tripletIndex = 0;

    for (hc::triplet const &tripletEl : triplets)
    {
        double const r = (double)((tripletIndex * 23) % 19) / 18.0;
        double const g = (double)((tripletIndex * 23) % 7) / 6.0;
        double const b = (double)((tripletIndex * 23) % 3) / 2.0;

        {
            auto const &pointA = (*cloud)[tripletEl.pointIndexA];
            auto const &pointB = (*cloud)[tripletEl.pointIndexB];

            std::ostringstream oss;
            oss << "line" << lineId;
            viewer.addLine<pcl::PointXYZI>(pointA, pointB, r, g, b, oss.str());
            ++lineId;
        }

        {
            auto const &pointA = (*cloud)[tripletEl.pointIndexB];
            auto const &pointB = (*cloud)[tripletEl.pointIndexC];

            std::ostringstream oss;
            oss << "line" << lineId;
            viewer.addLine<pcl::PointXYZI>(pointA, pointB, r, g, b, oss.str());
            ++lineId;
        }

        ++tripletIndex;
    }
}
#endif

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
#ifdef F17_VISUALIZE
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setPosition(0, 0);
    viewer.setSize(1920, 1000);
    viewer.setShowFPS(true);
    viewer.setBackgroundColor(1.0, 1.0, 1.0);
    viewer.setCameraPosition(0.0, 0.0, 2000.0, 0.0, 1.0, 0.0);
    viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
    viewer.registerPointPickingCallback([](pcl::visualization::PointPickingEvent const &event) { std::cout << event.getPointIndex() << std::endl; });
#endif

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

    /*
    // data03-spiral.pcd (1875)
    bestParams. = 50.0f;
    bestParams.smoothRadius = 8.88751f;
    bestParams.genTripletsNnKandidates = 16;
    bestParams.genTripletsNBest = 2;
    bestParams.genTripletsMaxError = 0.00316365f;
    bestParams.bestClusterDistanceDelta = 13.0245f;
    bestParams.cleanupMinTriplets = 33;

    // data13.pcd (152)
    bestParams.cloudScaleModifier = 50.0f;
    bestParams.smoothRadius = 7.77049f;
    bestParams.genTripletsNnKandidates = 23;
    bestParams.genTripletsNBest = 2;
    bestParams.genTripletsMaxError = 0.00133989f;
    bestParams.bestClusterDistanceDelta = 13.0245f;
    bestParams.cleanupMinTriplets = 33;

    // data40.pcd (75)
    bestParams.cloudScaleModifier = 50.0f;
    bestParams.smoothRadius = 10.4888f;
    bestParams.genTripletsNnKandidates = 15;
    bestParams.genTripletsNBest = 2;
    bestParams.genTripletsMaxError = 0.00898972f;
    bestParams.bestClusterDistanceDelta = 13.0245f;
    bestParams.cleanupMinTriplets = 33;

    // data41.pcd (150)
    bestParams.cloudScaleModifier = 50.0f;
    bestParams.smoothRadius = 10.4888f;
    bestParams.genTripletsNnKandidates = 13;
    bestParams.genTripletsNBest = 2;
    bestParams.genTripletsMaxError = 0.00898972f;
    bestParams.bestClusterDistanceDelta = 13.0245f;
    bestParams.cleanupMinTriplets = 26;

    // data44-spiral.pcd (149)
    bestParams.cloudScaleModifier = 50.0f;
    bestParams.smoothRadius = 17.8779f;
    bestParams.genTripletsNnKandidates = 20;
    bestParams.genTripletsNBest = 2;
    bestParams.genTripletsMaxError = 0.00807784f;
    bestParams.bestClusterDistanceDelta = 13.0245f;
    bestParams.cleanupMinTriplets = 26;

    // data49.pcd (673)
    bestParams.cloudScaleModifier = 50.0f;
    bestParams.smoothRadius = 37.8281f;
    bestParams.genTripletsNnKandidates = 20;
    bestParams.genTripletsNBest = 2;
    bestParams.genTripletsMaxError = 0.00898972f;
    bestParams.bestClusterDistanceDelta = 30.3917f;
    bestParams.cleanupMinTriplets = 26;

    // data62.pcd (232)
    bestParams.cloudScaleModifier = 50.0f;
    bestParams.smoothRadius = 10.3535f;
    bestParams.genTripletsNnKandidates = 21;
    bestParams.genTripletsNBest = 2;
    bestParams.genTripletsMaxError = 0.00898972f;
    bestParams.bestClusterDistanceDelta = 5.63541f;
    bestParams.cleanupMinTriplets = 6;

    // data99.pcd (0)
    bestParams.cloudScaleModifier = 50.0f;
    bestParams.smoothRadius = 10.3037f;
    bestParams.genTripletsNnKandidates = 14;
    bestParams.genTripletsNBest = 2;
    bestParams.genTripletsMaxError = 0.00898972f;
    bestParams.bestClusterDistanceDelta = 13.0245f;
    bestParams.cleanupMinTriplets = 6;

    // data190-spiral.pcd (4935)
    bestParams.cloudScaleModifier = 50.0f;
    bestParams.smoothRadius = 57.4456f;
    bestParams.genTripletsNnKandidates = 12;
    bestParams.genTripletsNBest = 2;
    bestParams.genTripletsMaxError = 0.00898972f;
    bestParams.bestClusterDistanceDelta = 35.8283f;
    bestParams.cleanupMinTriplets = 46;
    */


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
/*
    filenames.push_back("output.288.pcd");
    filenames.push_back("test.129.pcd");
    filenames.push_back("test.138.pcd");
    filenames.push_back("test.142.pcd");
    filenames.push_back("test.150.pcd");
    filenames.push_back("test.157.pcd");
    filenames.push_back("test.163.pcd");
    filenames.push_back("test.164.pcd");
    filenames.push_back("test.178.pcd");
    filenames.push_back("test.183.pcd");
    filenames.push_back("test.184.pcd");
    filenames.push_back("test.186.pcd");
    filenames.push_back("test.189.pcd");
    filenames.push_back("test.190.pcd");
    filenames.push_back("test.192.pcd");
    filenames.push_back("test.198.pcd");
    filenames.push_back("test.199.pcd");
    filenames.push_back("test.206.pcd");
    filenames.push_back("test.223.pcd");
    filenames.push_back("test.224.pcd");
    filenames.push_back("test.229.pcd");
    filenames.push_back("test.232.pcd");
    filenames.push_back("test.234.pcd");
    filenames.push_back("test.239.pcd");
    filenames.push_back("test.241.pcd");
    filenames.push_back("test.243.pcd");
    filenames.push_back("test.246.pcd");
    filenames.push_back("test.248.pcd");
    filenames.push_back("test.249.pcd");
    filenames.push_back("test.254.pcd");
    filenames.push_back("test.255.pcd");
    filenames.push_back("test.257.pcd");
    filenames.push_back("test.259.pcd");
    filenames.push_back("test.261.pcd");
    filenames.push_back("test.272.pcd");
    filenames.push_back("test.276.pcd");
    filenames.push_back("test.278.pcd");
    filenames.push_back("test.281.pcd");
    filenames.push_back("test.282.pcd");
*/

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
#ifdef F17_OPTIMIZE_PARAMETERS
    std::vector<Cluster> groundTruth;
#endif

    for (std::string const &filename : filenames)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::io::loadPCDFile("data/nscl-pcd-ex/" + filename, *cloud);
        //pcl::io::loadPCDFile("data/nscl-pcd-ex-new/" + filename, *cloud);

        clouds.push_back(cloud);
#ifdef F17_OPTIMIZE_PARAMETERS
        groundTruth.push_back(Cluster("data/ground-truth/" + filename + ".cl"));
#endif
    }

    while (true)
    {
        std::vector<hc_params> hcParams;

        hcParams.push_back(bestParams);

#ifdef F17_OPTIMIZE_PARAMETERS
        for (float offset = -10.0f; offset <= 5.0f; offset += 1.0f)
        {
            float offsetExp = std::exp(offset);

            {
                hc_params a = bestParams;
                a.cloudScaleModifier += offsetExp;
                hcParams.push_back(a);
            }

            {
                hc_params a = bestParams;
                a.cloudScaleModifier -= offsetExp;

                if (a.cloudScaleModifier >= 0.0f)
                    hcParams.push_back(a);
            }

            {
                hc_params a = bestParams;
                a.bestClusterDistanceDelta += offsetExp;
                hcParams.push_back(a);
            }

            {
                hc_params a = bestParams;
                a.bestClusterDistanceDelta -= offsetExp;

                if (a.bestClusterDistanceDelta >= 0.0f)
                    hcParams.push_back(a);
            }

            {
                hc_params a = bestParams;
                a.genTripletsMaxError += offsetExp;
                hcParams.push_back(a);
            }

            {
                hc_params a = bestParams;
                a.genTripletsMaxError -= offsetExp;

                if (a.genTripletsMaxError >= 0.0f)
                    hcParams.push_back(a);
            }

            {
                hc_params a = bestParams;
                a.smoothRadius += offsetExp;
                hcParams.push_back(a);
            }

            {
                hc_params a = bestParams;
                a.smoothRadius -= offsetExp;

                if (a.smoothRadius >= 0.0f)
                    hcParams.push_back(a);
            }
        }

        for (size_t offset = 0; offset <= 5; ++offset)
        {
            size_t offsetExp = (size_t)std::exp(offset);

            {
                hc_params a = bestParams;
                a.cleanupMinTriplets += offsetExp;
                hcParams.push_back(a);
            }

            if (bestParams.cleanupMinTriplets >= offsetExp)
            {
                hc_params a = bestParams;
                a.cleanupMinTriplets -= offsetExp;

                if (a.cleanupMinTriplets >= 0)
                    hcParams.push_back(a);
            }

            /*
            {
                hc_params a = bestParams;
                a.genTripletsNBest += offsetExp;
                hcParams.push_back(a);
            }

            if (bestParams.genTripletsNBest >= offsetExp)
            {
                hc_params a = bestParams;
                a.genTripletsNBest -= offsetExp;

                if (a.genTripletsNBest >= 1)
                    hcParams.push_back(a);
            }
            */

            {
                hc_params a = bestParams;
                a.genTripletsNnKandidates += offsetExp;
                hcParams.push_back(a);
            }

            if(bestParams.genTripletsNnKandidates >= offsetExp)
            {
                hc_params a = bestParams;
                a.genTripletsNnKandidates -= offsetExp;

                if (a.genTripletsNnKandidates >= 2)
                    hcParams.push_back(a);
            }
        }

        std::vector<int> paramErrors(hcParams.size());

#ifndef F17_VISUALIZE
        std::mutex stdOutProtector;
#pragma omp parallel for schedule(dynamic)
#endif
#endif
        for (int paramIndex = 0; paramIndex < (int)hcParams.size(); ++paramIndex)
        {
#ifdef F17_VISUALIZE
            if (viewer.wasStopped())
                break;
#endif

            hc_params const &currentHcParams = hcParams[paramIndex];
#ifdef F17_OPTIMIZE_PARAMETERS
            int totalError = 0;
#endif

            for (size_t fileIndex = 0; fileIndex < filenames.size(); ++fileIndex)
            {
#ifdef F17_VISUALIZE
                if (viewer.wasStopped())
                    break;

                do
                {
                    reload = false;
#endif
#ifdef F17_OPTIMIZE_PARAMETERS
                    Cluster const &currentGroundTruth = groundTruth[fileIndex];
#endif

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

#ifdef F17_USE_FILECACHE
                        try
                        {
                            // try to load cluster-data from file
                            cluster.load(currentFile + ".cl");
                        }
                        catch (...)
#endif
                        {
                            // calculate
                            triplets = hc::generateTriplets(cloud_xyzti_smooth, currentHcParams.genTripletsNnKandidates, currentHcParams.genTripletsNBest, currentHcParams.genTripletsMaxError);
                            cluster = useHc(cloud_xyzti_smooth, triplets, cloudScale * currentHcParams.cloudScaleModifier, currentHcParams.bestClusterDistanceDelta, currentHcParams.cleanupMinTriplets);
#ifdef F17_USE_FILECACHE
                            cluster.save(currentFile + ".cl");
#endif
                        }

#ifdef F17_VISUALIZE
                        // color-code
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>());

                        //colorByIndex(cloud_xyz, cloud_xyzrgb);
                        //colorByIntensity(cloud_xyzti, cloud_xyzrgb);
                        colorByCluster(cloud_xyz, cluster, cloud_xyzrgb);

                        // visualize
                        viewer.removeAllPointClouds();
                        viewer.removeAllShapes();

                        //visualizeTriplets(viewer, cloud_xyzti_smooth, triplets);
                        visualizeClusterAsMst(viewer, cloud_xyzti, cluster);

                        //viewer.addPointCloud(cloud_outliner, "cloud_outliner");
                        //viewer.addPointCloud(cloud_xyzti, "cloud_filtered");
                        viewer.addPointCloud(cloud_xyzrgb, "cloud");

                        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
                        //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.75, "cloud_filtered");
                        //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0.5, "cloud_filtered");
                        //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "cloud_outliner");
                        //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud_outliner");

                        wait = true;
                        while (wait && !viewer.wasStopped())
                        {
                            viewer.spinOnce(16);
                            std::this_thread::sleep_for(std::chrono::milliseconds(16));
                        }
#endif
#ifdef F17_OPTIMIZE_PARAMETERS
                        totalError += (currentGroundTruth - cluster);
#endif
                    }
#ifdef F17_VISUALIZE
                }
                while (reload && !viewer.wasStopped());
#endif
            }

#ifdef F17_OPTIMIZE_PARAMETERS
#ifndef F17_VISUALIZE
            stdOutProtector.lock();
#endif
            hcParamsResults << totalError << " " << paramIndex << " - "
                << "cloudScaleModifier=" << currentHcParams.cloudScaleModifier
                << ", smoothRadius=" << currentHcParams.smoothRadius
                << ", genTripletsNnKandidates=" << currentHcParams.genTripletsNnKandidates
                << ", genTripletsNBest=" << currentHcParams.genTripletsNBest
                << ", genTripletsMaxError=" << currentHcParams.genTripletsMaxError
                << ", bestClusterDistanceDelta=" << currentHcParams.bestClusterDistanceDelta
                << ", cleanupMinTriplets=" << currentHcParams.cleanupMinTriplets
                << std::endl;

            std::cout << totalError << ": " << (paramIndex + 1) << "/" << hcParams.size() << std::endl;
            paramErrors[paramIndex] = totalError;
#ifndef F17_VISUALIZE
            stdOutProtector.unlock();
#endif
#endif
        }

#ifdef F17_VISUALIZE
        viewer.close();
#endif
#ifdef F17_OPTIMIZE_PARAMETERS
        size_t smallestElement = std::distance(paramErrors.cbegin(), std::min_element(paramErrors.cbegin(), paramErrors.cend()));
        int bestError = paramErrors[smallestElement];
        bestParams = hcParams[smallestElement];

        std::cout << "BEST:" << std::endl
            << bestError << " (old: " << paramErrors[0] << ")" << std::endl
            << "cloudScaleModifier=" << bestParams.cloudScaleModifier << std::endl
            << "smoothRadius=" << bestParams.smoothRadius << std::endl
            << "genTripletsNnKandidates=" << bestParams.genTripletsNnKandidates << std::endl
            << "genTripletsNBest=" << bestParams.genTripletsNBest << std::endl
            << "genTripletsMaxError=" << bestParams.genTripletsMaxError << std::endl
            << "bestClusterDistanceDelta=" << bestParams.bestClusterDistanceDelta << std::endl
            << "cleanupMinTriplets=" << bestParams.cleanupMinTriplets << std::endl;
        
        // position 0 is the last bestParams
        if (smallestElement == 0)
            break;
#endif
    }

#ifdef F17_OPTIMIZE_PARAMETERS
    std::cout << "DONE!" << std::endl;
    std::cin.get();
#endif

    return 0;
}
