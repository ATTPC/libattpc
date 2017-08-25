/**
 * @file
 * @brief A set of common utility functions for the cleaning code.
 */

#ifndef ATTPC_CLEANING_CONVERSIONS_H
#define ATTPC_CLEANING_CONVERSIONS_H

#include "eigen_common.h"
#include <pcl/common/common.h>
#include <functional>
#include <algorithm>
#include <numeric>

namespace attpc {
namespace cleaning {

pcl::PointCloud<pcl::PointXYZI> pointCloudFromCArray(
        const float *const data,
        const int numRows,
        const int numCols);

template <class T>
Eigen::MatrixXf calculateDistanceMatrix(const std::vector<T>& points, const std::function<float(const T&, const T&)>& metric) {
    const size_t tripletSize = points.size();
    Eigen::MatrixXf result(tripletSize, tripletSize);

    for (Eigen::Index i = 0; i < tripletSize; ++i) {
        result(i, i) = 0.0f;

        for (Eigen::Index j = i + 1; j < tripletSize; ++j) {
            result(i, j) = metric(points[i], points[j]);
            result(j, i) = result(i, j);
        }
    }

    return result;
}

/**
 * @brief Find peaks in the given array of data.
 *
 * This function is based on the Python function scipy.signal.argrelmax from the SciPy library. It
 * returns the locations of the maxima in the data that are greater than or equal to at least `order`
 * points to each side.
 *
 * If the data includes flat-topped peaks, the location of the left edge of each peak will be returned (in other
 * words, the result will be the index of the first point in the flat top).
 *
 * Finally, a baseline of 0 is assumed, so all peaks must be greater than 0.
 *
 * @param data  The input data. It must be an array consisting of a single column. This is checked using a static
 *              assertion, so the number of columns in the array must be known at compile time.
 * @param order The order of the peak. For example, an order 3 peak is greater than or equal to the 3 points to
 *              either side of it.
 *
 * @return      The bin indices corresponding to the maxima of all of the peaks that were found.
 */
template <class Derived>
std::vector<Eigen::Index> findPeakLocations(const Eigen::DenseBase<Derived>& data, const Eigen::Index order = 1) {
    static_assert(Derived::ColsAtCompileTime == 1 && Derived::RowsAtCompileTime != 1,
                  "This function requires an array with a single column of data.");

    std::vector<Eigen::Index> peakLocs;

    for (Eigen::Index dataIdx = 0; dataIdx < data.rows(); ++dataIdx) {
        const auto currentPointValue = data(dataIdx);
        if (currentPointValue > 0) {  // Exclude zero values so the baseline doesn't count as a peak
            // firstPt and lastPt are set here to enforce array bounds
            const Eigen::Index firstPt = std::max(dataIdx - order, Eigen::Index{0});
            const Eigen::Index lastPt = std::min(dataIdx + order, data.rows() - 1);

            bool isPeak = true;
            for (Eigen::Index offsetIdx = firstPt; offsetIdx <= lastPt; ++offsetIdx) {
                isPeak &= data(offsetIdx) <= currentPointValue;
            }
            if (isPeak) {
                peakLocs.push_back(dataIdx);
            }
        }
    }

    // Filter out duplicate peaks from flat tops
    auto newEndIter = std::remove_if(peakLocs.begin(), peakLocs.end(), [&data](auto index) {
        // The first part of this predicate prevents us from requesting data(-1)
        // Also, if index zero is a peak, it should be kept since it can't be a later
        // part of a flat-top peak.
        return (index != 0) && (data(index) == data(index - 1));
    });
    peakLocs.erase(newEndIter, peakLocs.end());

    return peakLocs;
}

/**
 * @brief Set all values in an array to zero if they are below the given threshold.
 *
 * @param data      The data array.
 * @param threshold The threshold value. Any value less than this will be set to 0.
 */
template <class Derived>
inline void applyThreshold(Eigen::DenseBase<Derived>& data, const typename Derived::Scalar threshold) {
    for (Eigen::Index colIdx = 0; colIdx < data.cols(); ++colIdx) {
        for (Eigen::Index rowIdx = 0; rowIdx < data.rows(); ++rowIdx) {
            if (data(rowIdx, colIdx) < threshold) {
                data(rowIdx, colIdx) = 0;
            }
        }
    }
}

}
}

#endif //ATTPC_CLEANING_CONVERSIONS_H
