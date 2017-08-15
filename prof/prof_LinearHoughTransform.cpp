#include "LinearHoughTransform.h"
#include <Eigen/Core>
#include <iostream>
#include <chrono>
#include <vector>
#include <numeric>
#include <string>

int main(const int argc, const char** argv) {
    using clock = std::chrono::high_resolution_clock;
    namespace chr = std::chrono;

    Eigen::Index numPts = 5000;
    Eigen::Index numBins = 500;
    int numIters = 20;

    if (argc > 1 && std::string(argv[1]) == "-h") {
        std::cout << "arguments: numPoints numBins numIterations" << std::endl;
        return 0;
    }

    if (argc > 1) {
        numPts = std::atoi(argv[1]);
    }
    if (argc > 2) {
        numBins = std::atoi(argv[2]);
    }
    if (argc > 3) {
        numIters = std::atoi(argv[3]);
    }

    const double maxRadiusValue = 1000;

    attpc::cleaning::LinearHoughTransform trans {numBins, maxRadiusValue};

    Eigen::ArrayXXd testData {numPts, 2};
    for (Eigen::Index row = 0; row < numPts; ++row) {
        testData(row, 0) = row;
        testData(row, 1) = row;
    }

    std::cout << "Profiling Hough transform with " << numPts << " points, "
              << numBins << " bins, and "
              << numIters << " iterations." << std::endl;

    trans.findHoughSpace(testData.col(0), testData.col(1));

    std::vector<clock::duration> durations;
    for (int iterNum = 0; iterNum < numIters; ++iterNum) {
        auto begin = clock::now();
        auto result = trans.findHoughSpace(testData.col(0), testData.col(1));
        auto end = clock::now();
        durations.push_back(end - begin);
    }

    clock::duration totalDuration = std::accumulate(durations.begin(), durations.end(), clock::duration::zero());
    double totalDurationMs = chr::duration_cast<chr::milliseconds>(totalDuration).count();
    double meanTimeMs = totalDurationMs / numIters;

    std::cout << "Mean duration: " << meanTimeMs << " ms\n";

    auto minmaxResult = std::minmax_element(durations.begin(), durations.end());
    std::cout << "Min duration: " << chr::duration_cast<chr::milliseconds>(*minmaxResult.first).count() << " ms\n";
    std::cout << "Max duration: " << chr::duration_cast<chr::milliseconds>(*minmaxResult.second).count() << " ms\n";
}
