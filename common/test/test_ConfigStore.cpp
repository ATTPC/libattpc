#include "catch.hpp"
#include "attpc/common/ConfigStore.h"
#include <boost/optional/optional_io.hpp>
#include <string>
#include <Eigen/Core>

using attpc::common::ConfigStore;
using namespace std::literals::string_literals;

namespace {
    const std::string configFileName = "data/test_config.yml";
}

TEST_CASE("ConfigStore can load a YAML file") {
    ConfigStore config {};
    REQUIRE_NOTHROW(config.load(configFileName));

    SECTION("Can parse an int option") {
        boost::optional<int> value = config.getValue<int>("int_option");
        REQUIRE(value);       // Check that it's not none
        REQUIRE(value == 5);  // Check the value
    }

    SECTION("Can parse a float option") {
        boost::optional<float> value = config.getValue<float>("float_option");
        REQUIRE(value);       // Check that it's not none
        REQUIRE(value == 4.7f);  // Check the value
    }

    SECTION("Can parse a string option") {
        boost::optional<std::string> value = config.getValue<std::string>("string_option");
        REQUIRE(value);       // Check that it's not none
        REQUIRE(value == "string value"s);  // Check the value
    }

    SECTION("Can parse an array option") {
        boost::optional<Eigen::Vector3i> value = config.getValue<Eigen::Vector3i>("array_option");

        Eigen::Vector3i expected;
        expected << 1, 2, 3;

        REQUIRE(value);  // Check that it's not none
        REQUIRE(value == expected);
    }

    SECTION("Returns none for an option that is not present in the file") {
        boost::optional<int> value = config.getValue<int>("missing_option");
        REQUIRE_FALSE(value);
    }
}

TEST_CASE("ConfigStore can load a subconfig") {
    ConfigStore config {configFileName};
    boost::optional<ConfigStore> sub = config.getSubConfig("subconfig");

    bool success {sub};  // Must convert separately since REQUIRE must be able to output its argument to cout
    REQUIRE(success);

    SECTION("Subconfig has int option") {
        boost::optional<int> value = sub->getValue<int>("sub_int_option");
        REQUIRE(value);       // Check that it's not none
        REQUIRE(value == 1);  // Check the value
    }

    SECTION("Subconfig has string option") {
        boost::optional<std::string> value = sub->getValue<std::string>("sub_string_option");
        REQUIRE(value);       // Check that it's not none
        REQUIRE(value == "sub string"s);  // Check the value
    }
}
