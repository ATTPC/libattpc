#ifndef ATTPC_COMMON_YAML_CONVERSIONS_H
#define ATTPC_COMMON_YAML_CONVERSIONS_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <type_traits>

namespace attpc {
namespace common {

template <class T>
struct is_eigen_1d
    : std::integral_constant<bool, T::RowsAtCompileTime == 1 ||
                                   T::ColsAtCompileTime == 1> {};

template <class Derived, typename std::enable_if<is_eigen_1d<Derived>::value, int>::type = 0>
YAML::Node encode_eigen(const Eigen::DenseBase<Derived>& array) {
    YAML::Node node;
    for (Eigen::Index arrayIdx = 0; arrayIdx < array.size(); ++arrayIdx) {
        node.push_back(array(arrayIdx));
    }
    return node;
}

template <class Derived, typename std::enable_if<is_eigen_1d<Derived>::value, int>::type = 0>
bool decode_eigen(const YAML::Node& node, Eigen::DenseBase<Derived>& array) {
    if (node.IsSequence()) {
        array.derived().resize(static_cast<Eigen::Index>(node.size()));
        for (auto nodeIter = node.begin(); nodeIter != node.end(); ++nodeIter) {
            Eigen::Index arrayIdx = std::distance(node.begin(), nodeIter);
            array(arrayIdx) = nodeIter->as<typename Derived::Scalar>();
        }
        return true;
    }
    else {
        return false;
    }
}

}
}

namespace YAML {

template<class Scalar, int RowsAtCompileTime, int ColsAtCompileTime, int Options, int MaxRowsAtCompileTime, int MaxColsAtCompileTime>
struct convert<Eigen::Array<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>> {
    static YAML::Node encode(const Eigen::Array<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>& array) {
        return attpc::common::encode_eigen(array);
    }

    static bool decode(const YAML::Node& node, Eigen::Array<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>& array) {
        return attpc::common::decode_eigen(node, array);
    }
};

template<class Scalar, int RowsAtCompileTime, int ColsAtCompileTime, int Options, int MaxRowsAtCompileTime, int MaxColsAtCompileTime>
struct convert<Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>> {
    static YAML::Node encode(const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>& matrix) {
        return attpc::common::encode_eigen(matrix);
    }

    static bool decode(const YAML::Node& node, Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>& matrix) {
        return attpc::common::decode_eigen(node, matrix);
    }
};

}

#endif /* end of include guard: ATTPC_COMMON_YAML_CONVERSIONS_H */
