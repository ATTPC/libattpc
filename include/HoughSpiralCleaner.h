//
// Created by Joshua Bradt on 7/28/17.
//

#ifndef ATTPC_CLEANING_HOUGHSPIRALCLEANER_H
#define ATTPC_CLEANING_HOUGHSPIRALCLEANER_H

#include "eigen_common.h"
#include <vector>
#include <algorithm>
#include <numeric>
#include <limits>
#include <cassert>
#include <array>
#include "CircularHoughTransform.h"
#include "LinearHoughTransform.h"
#include "HoughSpace.h"
#include "utilities.h"

namespace attpc {
namespace cleaning {

/**
 * @brief Configuration container for the HoughSpiralCleaner class.
 */
class HoughSpiralCleanerConfig {
public:
    //! The number of bins to use in each dimension of the linear Hough space.
    Eigen::Index linearHoughNumBins;

    //! The maximum value in the radius dimension of the linear Hough space.
    double linearHoughMaxRadius;

    //! The number of bins to use in each dimension of the circular Hough space.
    Eigen::Index circularHoughNumBins;

    //! The maximum value in the radius dimension of the circular Hough space.
    double circularHoughMaxRadius;

    //! The number of angular bins to average over when finding the maximum angle in the Hough space.
    Eigen::Index numAngleBinsToReduce;

    //! The number of angular bins to include in the slice of the Hough space around the maximum angle.
    Eigen::Index houghSpaceSliceSize;

    //! The number of radial bins in each direction to include when finding the center of gravity of each peak.
    Eigen::Index peakWidth;

    //! The minimum number of points that must be included in each found line.
    int minPointsPerLine;

    //! Neighborhood radius used when determining how many neighbors each point has.
    double neighborRadius;
};

/**
 * @brief The result of the Hough spiral cleaning process.
 */
class HoughSpiralCleanerResult {
public:
    /**
     * @brief Default constructor
     *
     * Note that this constructor does not initialize any of the arrays contained by the object.
     */
    HoughSpiralCleanerResult() = default;
    /**
     * @brief Constructor that initializes the contained arrays.
     *
     * The labels array is initialized to -1 for each point, and the distancesToNearestLine array is initialized
     * to +infinity for each point. The neighborCounts are initialized to -1. The center array is not initialized.
     *
     * @param numPts The dimension of the labels and distancesToNearestLine arrays.
     */
    HoughSpiralCleanerResult(const Eigen::Index numPts);

    /**
     * @brief The label that identifies the line each point was assigned to.
     *
     * For a valid point, this should have a non-negative value. Noise points will be assigned
     * a label of -1.
     */
    Eigen::Array<Eigen::Index, Eigen::Dynamic, 1> labels;

    /**
     * @brief The vertical distance from each point to the nearest line.
     *
     * This distance is the vertical distance in the arclength vs. z space. Note that it **will be** defined for
     * every point, even if that point is labeled as a noise point. Cuts can be applied on this distance to
     * reduce the amount of noise in the dataset.
     */
    Eigen::ArrayXd distancesToNearestLine;

    /**
     * @brief The number of neighbors each point has inside the neighborhood radius.
     */
    Eigen::ArrayXi neighborCounts;

    //! The center of curvature (x, y) found by the circular Hough transform.
    Eigen::Vector2d center;
};

/**
 * @brief An implementation of the Hough transform cleaning algorithm for spiral-shaped tracks.
 *
 * This class uses the CircularHoughTransform and LinearHoughTransform classes to identify and tag
 * noise points in events that contain spiral-shaped tracks. An event can be cleaned using the
 * processEvent method, which will return a HoughSpiralCleanerResult object containing the label
 * assigned to each point and the distance from each point to the nearest line in the Hough space.
 * Each step of the process can also be called independently, if desired.
 */
class HoughSpiralCleaner {
public:
    using AngleSliceArrayType = Eigen::Array<HoughSpace::ScalarType, Eigen::Dynamic, 1>;

    /**
     * @brief Construct an instance using a configuration object.
     *
     * The internal parameters of the constructed instance are set using the values in the provided
     * HoughSpiralCleanerConfig object. See the documentation of that class for details about each
     * parameter.
     *
     * @param config The configuration object.
     */
    HoughSpiralCleaner(const HoughSpiralCleanerConfig& config);

    /**
     * @brief Identify and tag noise points in an event using the Hough transform cleaning algorithm.
     *
     * This method performs the entire calculation and returns the results. This is usually the method
     * that should be used to process the data.
     *
     * @param  xyz The (x, y, z) points of the event. These are assumed to occupy the first three columns
     *             of the array.
     * @return     The results of the cleaning algorithm. This includes the labels, the distance to each line,
     *             and the center of curvature found by the Hough transform for circles.
     */
    HoughSpiralCleanerResult processEvent(const Eigen::Ref<const Eigen::ArrayXXd>& xyz) const;

    /**
     * @brief Count the number of neighbors each point has.
     *
     * The neighborhood radius is controlled by the neighborRadius member of this class.
     *
     * @param  xyz The data, with columns corresponding to variables and rows corresponding to points.
     *             All columns will contribute to the calculation of the distance between points.
     * @return     The number of neighbors that each point has. Points are not counted as their own neighbors.
     */
    Eigen::ArrayXi countNeighbors(const Eigen::Ref<const Eigen::ArrayXXd>& xyz) const;

    /**
     * @brief Find the center of a spiral
     *
     * This wraps CircularHoughTransform::findCenter.
     *
     * @param  xs   The x coordinates of the data.
     * @param  ys   The y coordinates of the data.
     * @return      The center (x, y) of the spiral as a 2D vector.
     * @throws HoughTransform::TooFewPointsException If the number of rows in the data is less than rowOffset.
     */
    Eigen::Vector2d findCenter(const Eigen::Ref<const Eigen::ArrayXd>& xs,
                               const Eigen::Ref<const Eigen::ArrayXd>& ys) const;

    /**
     * @brief Calculate the arc length swept by the track in the xy projection.
     *
     * The arc length is calculated using an angle that is found with respect to the given center of curvature and
     * the x axis. The angles are calculated using atan2, so they have a branch cut at pi. This means that the arc
     * length will also have a branch cut at R*pi.
     *
     * @param  xy     The (x, y) data. These values are assumed to occupy the first two columns of the array.
     * @param  center The center of curvature (x, y).
     * @return        The arc lengths, a one-column array with the same number of rows as the input data.
     */
    Eigen::ArrayXd findArcLength(const Eigen::Ref<const Eigen::ArrayXXd>& xy, const Eigen::Vector2d& center) const;

    /**
     * @brief Calculate the linear Hough transform.
     *
     * This is simply a wrapper for the LinearHoughTransform object owned by this class.
     *
     * @param  zs      The z positions of the track. This will be the x variable in the Hough transform.
     * @param  arclens The arclength values of the track. This will be the y variable in the Hough transform.
     * @return         The Hough space.
     */
    HoughSpace findHoughSpace(const Eigen::Ref<const Eigen::ArrayXd>& zs,
                              const Eigen::Ref<const Eigen::ArrayXd>& arclens) const;

    /**
     * @brief Find the angle bin index of the largest maximum in the Hough space.
     *
     * To account for fluctuations, the maximum is found by averaging the angle bin index of the top N bins
     * in the Hough space, where N is the parameter numAngleBinsToReduce. This is set by the config object
     * when constructing this class.
     *
     * @param  houghSpace The Hough space to consider.
     * @return            The angle bin corresponding to the maximum.
     */
    Eigen::Index findMaxAngleBin(const HoughSpace& houghSpace) const;

    /**
     * @brief Extract a slice of the Hough space along the angular dimension around the given bin.
     *
     * A slice of the Hough space of width 2*houghSpaceSliceSize is taken around the given maxAngleBin. This slice
     * is then summed binwise to produce a 1D array with length equal to the number of bins in the linear Hough space.
     * Each element of the result therefore corresponds to a different radius bin in the Hough space.
     *
     * @param  houghSpace  The Hough space to slice.
     * @param  maxAngleBin The angle bin in the center of the slice.
     * @return             The summed slice, as described above.
     */
    AngleSliceArrayType findMaxAngleSlice(const HoughSpace& houghSpace, const Eigen::Index maxAngleBin) const;

    /**
     * @brief Find each radius peak in the given angular slice of the Hough space.
     *
     * The peaks are found using the center-of-gravity method, so the results are floating-point values even
     * though they refer to bin indices. The center of gravity is found by considering bins up to a distance
     * of peakWidth away in each direction from the maximum of a given peak.
     *
     * @param  houghSlice The slice of the Hough space. This should be one-dimensional.
     * @return            The center of gravity of each peak, as a (floating-point) bin index.
     */
    std::vector<double> findPeakRadiusBins(const Eigen::Ref<const AngleSliceArrayType>& houghSlice) const;

    /**
     * @brief Find the distance to the nearest line for each point, and identify the noise points.
     *
     * Note that this function takes actual angle and radius *values*, and **not** bin indices.
     *
     * @param  zs       The z coordinate of each point.
     * @param  arclens  The arclength coordinate of each point.
     * @param  maxAngle The angle value (not bin) corresponding to the maximum in the Hough space.
     * @param  radii    The radius value (not bin) peaks from the maximum angular slice of the Hough space.
     * @return          The result object, which contains the labels and the distances to the nearest line. Note
     *                  that this function does **not** set the center of curvature in the result object.
     */
    HoughSpiralCleanerResult classifyPoints(
        const Eigen::Ref<const Eigen::ArrayXd>& zs,
        const Eigen::Ref<const Eigen::ArrayXd>& arclens,
        const double maxAngle,
        const Eigen::Ref<const Eigen::ArrayXd>& radii) const;

private:
    Eigen::Index numAngleBinsToReduce;
    Eigen::Index houghSpaceSliceSize;
    Eigen::Index peakWidth;
    int minPointsPerLine;
    double neighborRadius;

    LinearHoughTransform linHough;
    CircularHoughTransform circHough;
};

}
}

#endif //ATTPC_CLEANING_HOUGHSPIRALCLEANER_H
