/**
 * @file
 * @brief Functions for counting the number of neighbors of each point in a data set.
 *
 * This file includes two functions for neighbor counting. The first, attpc::cleaning::countNeighbors, implements the
 * actual neighbor-counting algorithm. The second function, attpc::cleaning::applyNeighborCut, can be used with the
 * results of the first function to eliminate points that have too few neighbors.
 */
#ifndef ATTPC_CLEANING_NEIGHBORS_H
#define ATTPC_CLEANING_NEIGHBORS_H

#include "attpc/cleaning/eigen_common.h"

namespace attpc {
namespace cleaning {

/**
 * @brief Count the number of neighbors each point has inside a given radius.
 *
 * This uses a somewhat naive algorithm to find the Euclidean distance between each pair of
 * points and increment a counter array if the distance is less than the given neighborhood
 * radius. Points are not counted as their own neighbors.
 *
 * @note Each column is treated as a variable, and the distance is found as the squared norm
 * of the distance between rows, so **all** columns in the data will count toward the distance.
 *
 * @param  data   The data, which should have a row for each point and a column for each dimension.
 * @param  radius The size of the neighborhood, or the maximum distance between points that are considered
 *                to be neighbors.
 * @return        The number of neighbors each point has, as a column vector. It will have the same number of
 *                rows as the input data.
 */
template <class Derived>
Eigen::ArrayXi countNeighbors(const Eigen::MatrixBase<Derived>& data, const typename Derived::Scalar radius) {
    using DataScalar = typename Derived::Scalar;

    const DataScalar radSquared = radius * radius;

    Eigen::ArrayXi counts = Eigen::ArrayXi::Constant(data.rows(), -1);

    for (Eigen::Index thisRowIdx = 0; thisRowIdx < data.rows(); ++thisRowIdx) {
        const typename Derived::ConstRowXpr thisRow = data.row(thisRowIdx);

        for (Eigen::Index otherRowIdx = 0; otherRowIdx < data.rows(); ++otherRowIdx) {
            const DataScalar distSquared = (thisRow - data.row(otherRowIdx)).squaredNorm();
            if (distSquared < radSquared) {
                ++counts(thisRowIdx);
            }
        }
    }

    return counts;
}

/**
 * @brief Extract the subset of the data points that has enough neighbors.
 *
 * @param data              The full data set, where columns are variables and rows are data points.
 * @param counts            The number of neighbors each point has. This should have the same number of rows
 *                          as the data.
 * @param neighborThreshold The minimum number of neighbors. Points will be discarded if they have fewer neighbors
 *                          than this threshold value.
 * @return subset           The set of points that have enough neighbors to satisfy the cut. This will have the same
 *                          number of columns as the input, but will have fewer rows unless all points satisfied the
 *                          cut.
 */
template <class Derived>
Eigen::Array<typename Derived::Scalar, Eigen::Dynamic, Derived::ColsAtCompileTime>
applyNeighborCut(const Eigen::DenseBase<Derived>& data,
                 const Eigen::Ref<const Eigen::ArrayXi>& counts,
                 const int neighborThreshold) {
    using SubsetType = Eigen::Array<typename Derived::Scalar, Eigen::Dynamic, Derived::ColsAtCompileTime>;

    const Eigen::Array<bool, Eigen::Dynamic, 1> keep = counts >= neighborThreshold;
    const Eigen::Index numGoodPts = keep.count();

    SubsetType subset {numGoodPts, data.cols()};

    for (Eigen::Index origIdx = 0, subsetIdx = 0; origIdx < data.rows(); ++origIdx) {
        if (keep(origIdx)) {
            subset.row(subsetIdx++) = data.row(origIdx);
        }
    }

    return subset;
}

}
}

#endif /* end of include guard: ATTPC_CLEANING_NEIGHBORS_H */
