#include "conversions.h"

namespace hc {
    pcl::PointCloud<pcl::PointXYZI> pointCloudFromCArray(
            const float *const data,
            const int numRows,
            const int numCols) {
        pcl::PointCloud<pcl::PointXYZI> cloud;

        for (int row = 0; row < numRows; ++row) {
            const int dataOffset = row * numCols;

            pcl::PointXYZI pt;
            pt.x         = data[dataOffset];
            pt.y         = data[dataOffset + 1];
            pt.z         = data[dataOffset + 2];
            pt.intensity = data[dataOffset + 3];
            cloud.push_back(pt);
        }

        return cloud;
    }
}
