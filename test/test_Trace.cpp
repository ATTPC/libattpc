#include "catch.hpp"
#include "Trace.h"

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
