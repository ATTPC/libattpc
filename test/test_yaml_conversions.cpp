#include "catch.hpp"
#include "yaml_conversions.h"
#include <Eigen/Core>

namespace {

template <class Derived>
void compareArrayAndNode(const Eigen::DenseBase<Derived>& array, const YAML::Node& node) {
    REQUIRE(node.IsSequence());
    REQUIRE(node.size() == array.size());

    for (auto iter = node.begin(); iter != node.end(); ++iter) {
        Eigen::Index arrayIdx = std::distance(node.begin(), iter);
        REQUIRE(array(arrayIdx) == iter->as<typename Derived::Scalar>());
    }
}

}


TEST_CASE("Can convert Eigen array to YAML node") {
    Eigen::ArrayXi data {4};
    data << 1, 2, 3, 4;

    YAML::Node node = YAML::convert<decltype(data)>::encode(data);

    compareArrayAndNode(data, node);
}

TEST_CASE("Can convert Eigen matrix to YAML node") {
    Eigen::VectorXi data {4};
    data << 1, 2, 3, 4;

    YAML::Node node = YAML::convert<decltype(data)>::encode(data);

    compareArrayAndNode(data, node);
}

TEST_CASE("Can convert Eigen row vector to YAML node") {
    Eigen::RowVectorXi data {4};
    data << 1, 2, 3, 4;

    YAML::Node node = YAML::convert<decltype(data)>::encode(data);

    compareArrayAndNode(data, node);
}

TEST_CASE("Can convert YAML node to Eigen array") {
    YAML::Node node;
    for (int i = 0; i < 4; ++i) {
        node.push_back(i);
    }

    Eigen::ArrayXi data;
    bool success = YAML::convert<decltype(data)>::decode(node, data);
    REQUIRE(success);

    compareArrayAndNode(data, node);
}

TEST_CASE("Can convert YAML node to Eigen matrix") {
    YAML::Node node;
    for (int i = 0; i < 4; ++i) {
        node.push_back(i);
    }

    Eigen::VectorXi data;
    bool success = YAML::convert<decltype(data)>::decode(node, data);
    REQUIRE(success);

    compareArrayAndNode(data, node);
}

TEST_CASE("Can convert YAML node to Eigen row vector") {
    YAML::Node node;
    for (int i = 0; i < 4; ++i) {
        node.push_back(i);
    }

    Eigen::RowVectorXi data;
    bool success = YAML::convert<decltype(data)>::decode(node, data);
    REQUIRE(success);

    compareArrayAndNode(data, node);
}
