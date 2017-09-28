#include "catch.hpp"
#include "attpc/mergers/GRAWFile.h"

using namespace attpc::mergers;

TEST_CASE("Can read partial-readout frame", "[GRAWFile]") {
    GRAWFile file {"test_data/partial_readout_frame.graw"};
    GRAWFrame frame = file.readFrame();

    SECTION("Header is correct") {
        CHECK(frame.header.metaType.value == 8);
        CHECK(frame.header.frameSize.value == 0x89);
        CHECK(frame.header.dataSource.value == 0);
        CHECK(frame.header.frameType.value == 1);
        CHECK(frame.header.revision.value == 5);
        CHECK(frame.header.headerSize.value == 1);
        CHECK(frame.header.itemSize.value == 4);
        CHECK(frame.header.itemCount.value == 0x22'00);
        CHECK(frame.header.eventTime.value == 0x00'01'2c'50'f1'1c);
        CHECK(frame.header.eventIdx.value == 0x08);
        CHECK(frame.header.coboIdx.value == 4);
        CHECK(frame.header.asadIdx.value == 0);
        CHECK(frame.header.readOffset.value == 0);
        CHECK(frame.header.status.value == 0);
        CHECK(frame.header.multip_0.value == 0);
        CHECK(frame.header.multip_1.value == 0);
        CHECK(frame.header.multip_2.value == 0);
        CHECK(frame.header.multip_3.value == 0);
        CHECK(frame.header.windowOut.value == 20);
        CHECK(frame.header.lastCell_0.value == 477);
        CHECK(frame.header.lastCell_1.value == 477);
        CHECK(frame.header.lastCell_2.value == 477);
        CHECK(frame.header.lastCell_3.value == 391);

        // This just counts the number of set bits. A better test should be done.
        CHECK(frame.header.hitPat_0.value.count() == 5);
        CHECK(frame.header.hitPat_1.value.count() == 5);
        CHECK(frame.header.hitPat_2.value.count() == 6);
        CHECK(frame.header.hitPat_3.value.count() == 10);
    }

    SECTION("First two data items are correct") {
        const auto& item0 = frame.data.at(0);
        REQUIRE(item0.aget == 0);
        REQUIRE(item0.channel == 11);
        REQUIRE(item0.timeBucket == 0);
        REQUIRE(item0.sample == 0x137);

        const auto& item1 = frame.data.at(1);
        REQUIRE(item1.aget == 1);
        REQUIRE(item1.channel == 11);
        REQUIRE(item1.timeBucket == 0);
        REQUIRE(item1.sample == 0x1AC);
    }

    SECTION("Number of data items is correct") {
        REQUIRE(frame.header.itemCount.value == frame.data.size());
    }
}
