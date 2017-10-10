#include "catch.hpp"
#include "attpc/common/Trace.h"

using attpc::common::Trace;
using attpc::common::HardwareAddress;

TEST_CASE("Traces can be ordered", "[Trace][HardwareAddress][FullTraceEvent]") {
    Trace a {HardwareAddress{0, 0, 0, 0}};
    Trace b {HardwareAddress{1, 1, 1, 1}};
    Trace c {HardwareAddress{2, 2, 2, 2}};

    CHECK(a < b);
    CHECK_FALSE(b < a);
    CHECK(b < c);
    CHECK_FALSE(c < b);
    CHECK(a < c);
    CHECK_FALSE(c < a);
}

TEST_CASE("Traces can be compared for equality", "[Trace]") {
    Trace a {HardwareAddress{0, 0, 0, 0}, 5};

    SECTION("Equal traces are equal") {
        Trace b = a;
        REQUIRE(a == b);
    }

    SECTION("Unequal addresses make traces unequal") {
        Trace b {HardwareAddress{0, 1, 0, 1}, a.getPad(), a.getData()};
        REQUIRE_FALSE(a == b);
    }

    SECTION("Unequal pads make traces unequal") {
        Trace b {a.getHardwareAddress(), 100, a.getData()};
        REQUIRE_FALSE(a == b);
    }

    SECTION("Unequal data makes traces unequal") {
        Trace b = a;
        b(10) = 100;
        REQUIRE_FALSE(a == b);
    }
}
