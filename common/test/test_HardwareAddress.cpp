#include "catch.hpp"
#include "attpc/common/HardwareAddress.h"
#include <vector>
#include <algorithm>

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

TEST_CASE("HardwareAddress hash is unique for all relevant values", "[FullTraceEvent][HardwareAddress]") {
    using namespace attpc;

    std::vector<size_t> hashValues;

    for (coboid_type cobo = 0; cobo < 20; cobo++) {
        for (asadid_type asad = 0; asad < 4; asad++) {
            for (agetid_type aget = 0; aget < 3; aget++) {
                for (channelid_type channel = 0; channel < 68; channel++) {
                    HardwareAddress addr {cobo, asad, aget, channel};
                    hashValues.push_back(std::hash<HardwareAddress>()(addr));
                }
            }
        }
    }

    std::sort(hashValues.begin(), hashValues.end());
    auto firstRepeatedElement = std::adjacent_find(hashValues.begin(), hashValues.end());
    REQUIRE(firstRepeatedElement == hashValues.end());
}
