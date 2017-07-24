#ifndef HC_CONVERSIONS
#define HC_CONVERSIONS

#include <pcl/common/common.h>

namespace hc {
    pcl::PointCloud<pcl::PointXYZI> pointCloudFromCArray(
            const float *const data,
            const int numRows,
            const int numCols);
}

#endif
