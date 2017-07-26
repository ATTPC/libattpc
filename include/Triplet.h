//
// Created by Joshua Bradt on 7/24/17.
//

#ifndef ATTPC_CLEANING_TRIPLET_H
#define ATTPC_CLEANING_TRIPLET_H

#include <cstddef>
#include <Eigen/Core>

namespace attpc {
namespace cleaning {

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


#endif //ATTPC_CLEANING_TRIPLET_H
