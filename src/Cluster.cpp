#include "Cluster.h"

#include <fstream>
#include <limits>
#include <cctype>

#include <boost/algorithm/string/replace.hpp>

namespace attpc {
namespace cleaning {

void Cluster::calculateRelationshipMatrixIfNecessary() const {
    if (this->relationshipMatrix.size() == 0) {
        this->relationshipMatrix = Eigen::ArrayXXi(this->pointIndexCount, this->pointIndexCount);
        this->relationshipMatrix.fill(0);

        for (pcl::PointIndicesConstPtr const& cluster : this->getClusters()) {
            std::vector<int> const& indices = cluster->indices;
            size_t indicesSize = indices.size();

            for (size_t i = 0; i < indicesSize; ++i) {
                int indexI = indices[i];

                ++this->relationshipMatrix(indexI, indexI);

                for (size_t j = i + 1; j < indicesSize; ++j) {
                    int indexJ = indices[j];

                    if (indexI < indexJ)
                        ++this->relationshipMatrix(indexI, indexJ);
                    else
                        ++this->relationshipMatrix(indexJ, indexI);
                }
            }
        }
    }
}

Cluster::Cluster(std::vector<pcl::PointIndicesPtr> const& clusters, size_t pointIndexCount) {
    this->clusters = clusters;
    this->pointIndexCount = pointIndexCount;
}

std::vector<pcl::PointIndicesPtr> const& Cluster::getClusters() const {
    return this->clusters;
}

size_t Cluster::getPointIndexCount() const {
    return this->pointIndexCount;
}

int Cluster::operator-(Cluster const& rhs) const {
    if (this->getPointIndexCount() != rhs.getPointIndexCount())
        throw std::runtime_error("pointIndexCount has to be identical!");

    this->calculateRelationshipMatrixIfNecessary();
    rhs.calculateRelationshipMatrixIfNecessary();

    return (this->relationshipMatrix - rhs.relationshipMatrix).abs().sum();
}

}
}
