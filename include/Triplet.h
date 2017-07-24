//
// Created by Joshua Bradt on 7/24/17.
//

#ifndef ATTPC_CLUSTERING_TRIPLET_H
#define ATTPC_CLUSTERING_TRIPLET_H

#include <cstddef>
#include <Eigen/Core>

namespace attpc {
namespace clustering {

class Triplet {
public:
    size_t pointIndexA;
    size_t pointIndexB;
    size_t pointIndexC;
    Eigen::Vector3f center;
    Eigen::Vector3f direction;
    float error;
};

}
}


#endif //ATTPC_CLUSTERING_TRIPLET_H
