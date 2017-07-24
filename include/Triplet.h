//
// Created by Joshua Bradt on 7/24/17.
//

#ifndef CLOUD_VIEWER_TRIPLET_H
#define CLOUD_VIEWER_TRIPLET_H

#include <cstddef>
#include <Eigen/Core>

namespace hc
{
    class Triplet
    {
    public:
        size_t pointIndexA;
        size_t pointIndexB;
        size_t pointIndexC;
        Eigen::Vector3f center;
        Eigen::Vector3f direction;
        float error;
    };
}


#endif //CLOUD_VIEWER_TRIPLET_H
