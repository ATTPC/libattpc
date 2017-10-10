#include "catch.hpp"
#include "attpc/common/FullTraceEvent.h"
#include <vector>

using attpc::common::FullTraceEvent;
using attpc::common::Trace;
using attpc::common::HardwareAddress;

TEST_CASE("Can insert traces into FullTraceEvent", "[FullTraceEvent]") {
    FullTraceEvent evt {};

    SECTION("Unique traces can be inserted") {
        std::vector<Trace> traces;
        traces.emplace_back(HardwareAddress{0, 1, 2, 3});
        traces.emplace_back(HardwareAddress{2, 3, 4, 5});
        traces.emplace_back(HardwareAddress{3, 4, 5, 6});

        for (const auto& trace : traces) {
            FullTraceEvent::iterator iter;
            bool wasInserted;
            std::tie(iter, wasInserted) = evt.insertTrace(trace);

            REQUIRE(wasInserted);
            REQUIRE(iter->getHardwareAddress() == trace.getHardwareAddress());
        }
    }

    SECTION("Non-unique traces fail to be inserted") {
        Trace trA {HardwareAddress{0, 1, 2, 3}};
        Trace trB = trA;

        bool wasInserted;
        FullTraceEvent::iterator iter;

        std::tie(iter, wasInserted) = evt.insertTrace(trA);
        REQUIRE(wasInserted);
        REQUIRE(iter->getHardwareAddress() == trA.getHardwareAddress());

        std::tie(iter, wasInserted) = evt.insertTrace(trB);
        REQUIRE_FALSE(wasInserted);
        REQUIRE(iter->getHardwareAddress() == trA.getHardwareAddress());
    }

    SECTION("Traces can be replaced if they already exist in the event") {
        Trace trA {HardwareAddress{0, 1, 2, 3}};
        trA(0) = 10;

        evt.insertTrace(trA);

        Trace trB {trA.getHardwareAddress()};
        trB(0) = 20;

        FullTraceEvent::iterator iter = evt.insertOrReplaceTrace(trB);
        REQUIRE(iter->getHardwareAddress() == trB.getHardwareAddress());
        REQUIRE(evt.numTraces() == 1);
        REQUIRE((*iter)(0) == trB(0));
    }
}

TEST_CASE("Can iterate through traces in FullTraceEvent", "[FullTraceEvent]") {
    std::vector<Trace> traces;
    for (attpc::coboid_type i = 0; i < 10; i++) {
        traces.emplace_back(HardwareAddress{i, 0, 0, 0});
    }

    FullTraceEvent evt {};
    for (const auto& trace : traces) {
        bool wasInserted;
        std::tie(std::ignore, wasInserted) = evt.insertTrace(trace);
        REQUIRE(wasInserted);
    }

    SECTION("Can iterate with iterators") {
        auto traceIter = traces.begin();
        for (FullTraceEvent::iterator evtIter = evt.begin(); evtIter != evt.end(); ++evtIter) {
            REQUIRE(evtIter->getHardwareAddress() == traceIter->getHardwareAddress());
            ++traceIter;
        }
    }

    SECTION("Can iterate with const iterators") {
        auto traceIter = traces.cbegin();
        for (FullTraceEvent::const_iterator evtIter = evt.cbegin(); evtIter != evt.cend(); ++evtIter) {
            REQUIRE(evtIter->getHardwareAddress() == traceIter->getHardwareAddress());
            ++traceIter;
        }
    }

    SECTION("Can iterate with range-based for loop") {
        auto traceIter = traces.begin();
        for (const auto& trace : evt) {
            REQUIRE(trace.getHardwareAddress() == traceIter->getHardwareAddress());
            ++traceIter;
        }
    }
}

TEST_CASE("Can find traces in FullTraceEvent", "[FullTraceEvent]") {
    FullTraceEvent evt {};

    std::vector<Trace> traces;
    for (attpc::coboid_type i = 0; i < 10; i++) {
        traces.emplace_back(HardwareAddress{i, 0, 0, 0});
        evt.insertTrace(traces.back());
    }

    SECTION("Can find traces that are present") {
        for (const Trace& trace : traces) {
            FullTraceEvent::iterator iter = evt.findTrace(trace.getHardwareAddress());
            REQUIRE(iter != evt.end());
            REQUIRE(iter->getHardwareAddress() == trace.getHardwareAddress());
        }
    }

    SECTION("Finding absent traces returns end") {
        Trace absentTrace {HardwareAddress{10, 10, 10, 10}};
        FullTraceEvent::iterator iter = evt.findTrace(absentTrace.getHardwareAddress());
        REQUIRE(iter == evt.end());
    }
}
