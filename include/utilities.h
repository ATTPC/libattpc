#ifndef ATTPC_CLEANING_CONVERSIONS_H
#define ATTPC_CLEANING_CONVERSIONS_H

#include <pcl/common/common.h>
#include <Eigen/Core>
#include <memory>
#include <functional>

namespace attpc {
namespace cleaning {

pcl::PointCloud<pcl::PointXYZI> pointCloudFromCArray(
        const float *const data,
        const int numRows,
        const int numCols);

template <class T>
Eigen::MatrixXf calculateDistanceMatrix(std::vector<T> const& points, const std::function<float(const T&, const T&)>& metric) {
    size_t const tripletSize = points.size();
    Eigen::MatrixXf result(tripletSize, tripletSize);

    for (size_t i = 0; i < tripletSize; ++i) {
        result(i, i) = 0.0f;

        for (size_t j = i + 1; j < tripletSize; ++j) {
            result(i, j) = metric(points[i], points[j]);
            result(j, i) = result(i, j);
        }
    }

    return result;
}

}
}

#endif //ATTPC_CLEANING_CONVERSIONS_H
