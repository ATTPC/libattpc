#include "catch.hpp"
#include "RawFrame.h"
#include <iterator>
#include <vector>

using attpc::mergers::RawFrame;

TEST_CASE("RawFrame iterators work correctly", "[RawFrame]") {
    const size_t frameSize = 20;
    RawFrame frame {frameSize};

    SECTION("end > begin") {
        REQUIRE(frame.end() > frame.begin());
        REQUIRE(frame.cend() > frame.cbegin());
    }

    SECTION("Works with std::distance") {
        REQUIRE(std::distance(frame.begin(), frame.end()) == frame.size());
        REQUIRE(std::distance(frame.cbegin(), frame.cend()) == frame.size());
    }

    SECTION("Size is correct") {
        REQUIRE(frame.size() == frameSize);
    }
}
