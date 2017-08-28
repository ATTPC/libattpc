#include "catch.hpp"
#include "ConfigStore.h"
#include <boost/optional/optional_io.hpp>
#include <string>

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

    SECTION("Returns none for an option that is not present in the file") {
        boost::optional<int> value = config.getValue<int>("missing_option");
        REQUIRE_FALSE(value);
    }
}
