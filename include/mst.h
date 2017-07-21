#pragma once

#include <vector>
#include <pcl/io/io.h>

namespace mst
{
    struct edge {
        size_t voxelIndexA, voxelIndexB;
        float distance;
    };

    struct state {
        std::vector<edge> edges;
        std::vector<size_t> groups;
    };

    typedef std::function<float(pcl::PointXYZI const &, pcl::PointXYZI const &, size_t, size_t, pcl::PointCloud<pcl::PointXYZI>::ConstPtr)> MstMetric;
    typedef std::function<bool(pcl::PointCloud<pcl::PointXYZI>::ConstPtr, edge const &, std::vector<edge> const &, std::vector<size_t> const &)> MstFilter;


    inline float defaultMstMetric(pcl::PointXYZI const &pointA, pcl::PointXYZI const &pointB, size_t indexA, size_t indexB, pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
    {
        Eigen::Vector3f diff(
            pointA.x - pointB.x,
            pointA.y - pointB.y,
            pointA.z - pointB.z
        );

        return diff.dot(diff);
    }

    inline bool defaultMstFilter(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, edge const &edgeV, std::vector<edge> const &selecetedEdges, std::vector<size_t> const &groups)
    {
        return true;
    }

    std::vector<edge> calculateSquareDistances(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, MstMetric metric = defaultMstMetric);
    std::vector<state> calculateMinimumSpanningTree(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, MstMetric metric = defaultMstMetric, MstFilter filter = defaultMstFilter);
}
