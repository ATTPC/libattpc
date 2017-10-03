#include "catch.hpp"
#include "attpc/common/PadLookupTable.h"
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>

namespace {

using attpc::common::PadLookupTable;
using attpc::common::HardwareAddress;

}

TEST_CASE("Can add elements to PadLookupTable", "[PadLookupTable]") {
    PadLookupTable table;
    HardwareAddress knownAddr {0, 1, 2, 3};
    HardwareAddress unknownAddr {1, 1, 1, 1};
    attpc::padid_type knownPad = 10;
    attpc::padid_type unknownPad = 40;

    table.insert(knownAddr, knownPad);

    SECTION("Can find known address") {
        boost::optional<attpc::padid_type> result = table.find(knownAddr);
        REQUIRE(result);
        REQUIRE(*result == knownPad);
    }

    SECTION("Can reverse find known pad number") {
        boost::optional<HardwareAddress> result = table.reverseFind(knownPad);
        REQUIRE(result);
        REQUIRE(*result == knownAddr);
    }

    SECTION("Finding unknown address returns boost::none") {
        boost::optional<attpc::padid_type> result = table.find(unknownAddr);
        REQUIRE_FALSE(result);
    }

    SECTION("Reverse finding unknown pad returns boost::none") {
        boost::optional<HardwareAddress> result = table.reverseFind(unknownPad);
        REQUIRE_FALSE(result);
    }
}

TEST_CASE("Can read lookup table from CSV file", "[PadLookupTable]") {
    PadLookupTable table {"data/test_lookup.csv"};

    SECTION("Known addresses are present") {
        boost::optional<attpc::padid_type> result0 = table.find({0, 1, 2, 3});
        REQUIRE(result0);
        REQUIRE(*result0 == 10);

        boost::optional<attpc::padid_type> result1 = table.find({1, 2, 3, 4});
        REQUIRE(result1);
        REQUIRE(*result1 == 20);
    }

    SECTION("Entries with -1 are ignored") {
        boost::optional<attpc::padid_type> result = table.find({-1, -1, -1, -1});
        REQUIRE_FALSE(result);
    }
}
