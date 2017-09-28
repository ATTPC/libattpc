#include "catch.hpp"
#include "attpc/common/HardwareAddress.h"
#include <vector>

using attpc::common::HardwareAddress;

TEST_CASE("HardwareAddress instances can be tested for equality", "[FullTraceEvent][HardwareAddress]") {
    SECTION("Equal addresses are equal") {
        HardwareAddress a {0, 1, 2, 3};
        HardwareAddress b {0, 1, 2, 3};
        REQUIRE(a == b);
    }

    SECTION("Unequal addresses are unequal") {
        HardwareAddress base {0, 1, 2, 3};

        std::vector<HardwareAddress> others;
        others.emplace_back(5, 1, 2, 3);
        others.emplace_back(0, 5, 2, 3);
        others.emplace_back(0, 1, 5, 3);
        others.emplace_back(0, 1, 2, 5);

        for (const auto& other : others) {
            REQUIRE_FALSE(base == other);
        }
    }
}

TEST_CASE("HardwareAddress instances can be compared with less-than operator", "[FullTraceEvent][HardwareAddress]") {
    HardwareAddress base {1, 1, 1, 1};

    SECTION("Valid on CoBo level") {
        REQUIRE_FALSE((base < HardwareAddress{0, 1, 1, 1}));
        REQUIRE      ((base < HardwareAddress{2, 1, 1, 1}));
    }

    SECTION("Valid on AsAd level") {
        REQUIRE_FALSE((base < HardwareAddress{1, 0, 1, 1}));
        REQUIRE      ((base < HardwareAddress{1, 2, 1, 1}));
    }

    SECTION("Valid on AGET level") {
        REQUIRE_FALSE((base < HardwareAddress{1, 1, 0, 1}));
        REQUIRE      ((base < HardwareAddress{1, 1, 2, 1}));
    }

    SECTION("Valid on channel level") {
        REQUIRE_FALSE((base < HardwareAddress{1, 1, 1, 0}));
        REQUIRE      ((base < HardwareAddress{1, 1, 1, 2}));
    }
}
