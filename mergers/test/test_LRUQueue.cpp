#include "catch.hpp"
#include "attpc/mergers/LRUQueue.h"
#include <boost/optional.hpp>
#include <boost/optional/optional_io.hpp>
#include <vector>
#include <random>
#include <algorithm>

using attpc::mergers::LRUQueue;

TEST_CASE("Can insert and extract items from LRUQueue", "[LRUQueue]") {
    using key_type = int;
    using value_type = int;

    LRUQueue<key_type, value_type> lq;

    REQUIRE(lq.size() == 0);
    REQUIRE(lq.empty());

    const key_type key = 5;
    const value_type value = 20;
    lq.insert(key, value);

    SECTION("Size was updated, and container isn't empty") {
        REQUIRE(lq.size() == 1);
        REQUIRE_FALSE(lq.empty());
    }

    SECTION("Can get reference to item") {
        boost::optional<value_type&> item = lq.get(key);
        REQUIRE(item);
        REQUIRE(*item == value);
        REQUIRE(lq.size() == 1);  // Check item was not deleted from queue
    }

    SECTION("Can extract item from queue") {
        boost::optional<value_type> item = lq.extract(key);
        REQUIRE(item);
        REQUIRE(*item == value);
        REQUIRE(lq.size() == 0);
        REQUIRE(lq.empty());
    }
}

TEST_CASE("Items are arranged in LRU order", "[LRUQueue]") {
    using key_type = int;
    using value_type = int;

    LRUQueue<key_type, value_type> lq;

    std::vector<std::pair<key_type, value_type>> items;
    for (key_type i = 0; i < 10; ++i) {
        items.emplace_back(i, i*2 + 1);
    }
    for (const auto& item : items) {
        lq.insert(item.first, item.second);
    }
    REQUIRE(lq.size() == items.size());

    // At this point, the first item in `items` should be the oldest one.
    // Permute the items in the LRUQueue by accessing them in some random order.
    std::mt19937 rng {0};
    std::shuffle(items.begin(), items.end(), rng);
    for (const auto& item : items) {
        auto value = lq.get(item.first);
        REQUIRE(value);
        REQUIRE(*value == item.second);
    }

    // Now check the order by removing the oldest elements until the LRUQueue is empty
    for (auto itemIter = items.begin(); itemIter != items.end() && !lq.empty(); ++itemIter) {
        std::pair<key_type, value_type> lqItem = lq.extractOldest();
        REQUIRE(lqItem == *itemIter);

        const size_t numItemsRemoved = static_cast<size_t>(std::distance(items.begin(), itemIter) + 1);
        const size_t expectedQueueSize = items.size() - numItemsRemoved;
        REQUIRE(lq.size() == expectedQueueSize);
    }

    REQUIRE(lq.empty());  // Make sure we checked every item
}
