#ifndef ATTPC_CLUSTERING_CONVERSIONS_H
#define ATTPC_CLUSTERING_CONVERSIONS_H

#include <pcl/common/common.h>

namespace attpc {
namespace clustering {

pcl::PointCloud<pcl::PointXYZI> pointCloudFromCArray(
        const float *const data,
        const int numRows,
        const int numCols);

}
}

#endif //ATTPC_CLUSTERING_CONVERSIONS_H
