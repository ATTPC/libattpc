#include "catch.hpp"
#include "utilities.h"
#include <vector>
#include <bitset>

using namespace attpc::mergers::utilities;

TEST_CASE("parseValue performs byte-swap", "[utilities]") {
    SECTION("Number of bytes == size of integer") {
        std::vector<uint8_t> raw = {0xAA, 0xBB, 0xCC, 0xDD};
        const uint32_t result = parseValue<uint32_t>(raw.begin(), raw.end());
        REQUIRE(result == 0xAABBCCDD);
    }

    SECTION("Number of bytes < size of integer") {
        std::vector<uint8_t> raw = {0xAA, 0xBB};
        const uint32_t result = parseValue<uint32_t>(raw.begin(), raw.end());
        REQUIRE(result == 0x0000AABB);
    }

    SECTION("Works with bitsets") {
        std::vector<uint8_t> raw = {0xAA, 0xBB, 0xCC, 0xDD};
        const std::bitset<32> result = parseValue<std::bitset<32>>(raw.begin(), raw.end());
        REQUIRE(result == 0xAABBCCDD);
    }
}
