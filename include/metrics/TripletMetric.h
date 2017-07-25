//
// Created by Joshua Bradt on 7/24/17.
//

#ifndef ATTPC_CLUSTERING_TRIPLETMETRIC_H
#define ATTPC_CLUSTERING_TRIPLETMETRIC_H

#include "Triplet.h"

namespace attpc {
namespace clustering {

class TripletMetric {
public:
    virtual float
    operator()(Triplet const& lhs, Triplet const& rhs) = 0;
};

}
}

#endif //ATTPC_CLUSTERING_TRIPLETMETRIC_H
