#include "catch.hpp"
#include "attpc/mergers/GRAWHeaderField.h"
#include "attpc/mergers/RawFrame.h"
#include <vector>
#include <algorithm>
#include <cstdint>
#include <type_traits>

using namespace attpc::mergers;

TEST_CASE("Can extract value from raw frame", "[GRAWHeaderField]") {
    const size_t offset = 4;
    const size_t size = 4;

    RawFrame frame {20};
    std::vector<RawFrame::byte_type> bytes = {0xAA, 0xBB, 0xCC, 0xDD};
    std::copy(bytes.begin(), bytes.end(), frame.begin() + offset);

    GRAWHeaderField<uint32_t, offset, size> field {frame};

    REQUIRE(field.value == 0xAABBCCDD);
}

TEST_CASE("Extraction throws exception when offset is too large", "[GRAWHeaderField]") {
    const size_t offset = 100;
    const size_t size = 4;

    RawFrame frame {20};

    REQUIRE_THROWS((GRAWHeaderField<uint32_t, offset, size> {frame}));
}
