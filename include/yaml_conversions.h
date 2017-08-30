/**
 * @file
 * @brief Additional conversions to and from YAML nodes
 *
 * These are used by the attpc::common::ConfigStore object (and by the underlying yaml-cpp library)
 * to encode and decode values from a config file as different data types. Each new conversion should be
 * a specialization of the struct template `YAML::convert`. Static functions `encode` and
 * `decode` can be provided in that specialization to handle the conversion. See the yaml-cpp tutorial
 * for more details.
 */

#ifndef ATTPC_COMMON_YAML_CONVERSIONS_H
#define ATTPC_COMMON_YAML_CONVERSIONS_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <type_traits>

namespace attpc {
namespace common {
namespace internal {

/**
 * @brief Statically check if the given type is a 1D Eigen Array or Matrix.
 *
 * If the type is 1D, the (static bool) member constant `value` will be true. Otherwise
 * it will be false.
 *
 * This will not work if the type is not known to be 1D at compile time.
 *
 * @tparam T The type to check.
 */
template <class T>
struct is_eigen_1d
    : std::integral_constant<bool, T::RowsAtCompileTime == 1 ||
                                   T::ColsAtCompileTime == 1> {};

/**
 * @brief Convert an Eigen array or matrix to a YAML node.
 *
 * This only works for arrays or matrices that are known to be 1D at compile time. The
 * restriction is enforced with std::enable_if, so attempting to use this function with
 * a 2D array should cause a compilation error.
 *
 * @note You should not need to call this function directly. It is used internally by the
 * yaml-cpp library.
 *
 * @param  array The 1D array to convert.
 * @return       The array as a YAML node.
 */
template <class Derived, typename std::enable_if<is_eigen_1d<Derived>::value, int>::type = 0>
YAML::Node encode_eigen(const Eigen::DenseBase<Derived>& array) {
    YAML::Node node;
    for (Eigen::Index arrayIdx = 0; arrayIdx < array.size(); ++arrayIdx) {
        node.push_back(array(arrayIdx));
    }
    return node;
}

/**
 * @brief Convert a YAML node to an Eigen array or matrix.
 *
 * This only works for arrays or matrices that are known to be 1D at compile time. The
 * restriction is enforced with std::enable_if, so attempting to use this function with
 * a 2D array should cause a compilation error.
 *
 * @note You should not need to call this function directly. It is used internally by the
 * yaml-cpp library.
 *
 * @warning Any data contained in the array before calling this function will be lost.
 *
 * @param  node  The node to decode.
 * @param  array The destination array or matrix. It will be resized to fit the data in the node.
 * @return       True on success, false on failure.
 */
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
}

namespace YAML {

//! @brief YAML conversion wrapper for Eigen arrays
template<class Scalar, int RowsAtCompileTime, int ColsAtCompileTime, int Options, int MaxRowsAtCompileTime, int MaxColsAtCompileTime>
struct convert<Eigen::Array<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>> {
    static YAML::Node encode(const Eigen::Array<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>& array) {
        return attpc::common::internal::encode_eigen(array);
    }

    static bool decode(const YAML::Node& node, Eigen::Array<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>& array) {
        return attpc::common::internal::decode_eigen(node, array);
    }
};

//! @brief YAML conversion wrapper for Eigen matrices
template<class Scalar, int RowsAtCompileTime, int ColsAtCompileTime, int Options, int MaxRowsAtCompileTime, int MaxColsAtCompileTime>
struct convert<Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>> {
    static YAML::Node encode(const Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>& matrix) {
        return attpc::common::internal::encode_eigen(matrix);
    }

    static bool decode(const YAML::Node& node, Eigen::Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime, Options, MaxRowsAtCompileTime, MaxColsAtCompileTime>& matrix) {
        return attpc::common::internal::decode_eigen(node, matrix);
    }
};

}

#endif /* end of include guard: ATTPC_COMMON_YAML_CONVERSIONS_H */
