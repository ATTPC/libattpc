#ifndef ATTPC_MERGERS_LRUQUEUE_H
#define ATTPC_MERGERS_LRUQUEUE_H

#include <list>
#include <unordered_map>
#include <utility>
#include <boost/optional.hpp>
#include <cassert>

namespace attpc {
namespace mergers {

/**
 * @brief A queue/cache that keeps track of objects in order of how recently they were accessed.
 *
 * Whenever an object in this queue is accessed, it is effectively moved to the front of the queue. This way,
 * the oldest object can easily be removed when the queue grows too large.
 *
 * @tparam  Key The type used as a key for retrieving objects
 * @tparam  T   The type of the stored objects
 */
template <class Key, class T>
class LRUQueue
{
public:
    //! The type of the key used to index the objects
    using key_type = Key;
    //! The type of the values stored in the queue
    using value_type = T;
    //! The type of a (key, value) pair
    using pair_type = std::pair<key_type, value_type>;

    /**
     * @brief Insert a value into the queue
     * @param  key   The key for retrieving the value later
     * @param  value The value to store
     * @return       A reference to the value stored in the queue
     */
    value_type& insert(const key_type& key, const value_type& value) {
        itemList.emplace_front(key, value);
        itemMap.emplace(key, itemList.begin());
        return itemList.front().second;
    }

    /**
     * @brief Get a reference to a value in the queue using its key.
     *
     * This will move this value to the front of the queue, and the value will remain in the queue.
     * For comparison, calling LRUQueue::extract will remove the value from the queue.
     *
     * @param key The key corresponding to the desired value.
     * @return    A reference to the value in the queue, or boost::none if the key was not found.
     */
    boost::optional<value_type&> get(const key_type& key) {
        auto mapIter = itemMap.find(key);
        if (mapIter == itemMap.end()) {
            return boost::none;
        }

        const key_type& foundKey = mapIter->first;
        assert(foundKey == key);
        const typename std::list<pair_type>::iterator& listIter = mapIter->second;

        // Move this item to the front of the list
        itemList.splice(itemList.begin(), itemList, listIter);

        value_type& item = itemList.front().second;

        return item;
    }

    /**
     * @brief Find a value in the queue, remove it, and return it.
     *
     * In comparison to LRUQueue::get, this removes the value from the queue before returning it.
     *
     * @param  key The key associated with the desired value.
     * @return     The value, or boost::none if the key was not found.
     */
    boost::optional<value_type> extract(const key_type& key) {
        auto mapIter = itemMap.find(key);
        if (mapIter == itemMap.end()) {
            return boost::none;
        }

        const key_type foundKey = mapIter->first;
        assert(foundKey == key);
        typename std::list<pair_type>::iterator listIter = mapIter->second;

        value_type item = std::move(listIter->second);
        itemList.erase(listIter);
        itemMap.erase(key);
        return item;
    }

    /**
     * @brief Extracts the least-recently accessed value from the queue.
     *
     * The value is removed from the queue, much like in LRUQueue::extract.
     *
     * @return The (key, value) pair corresponding to the oldest item.
     */
    pair_type extractOldest() {
        pair_type oldest = itemList.back();
        itemList.pop_back();

        const key_type key = oldest.first;
        itemMap.erase(key);

        return oldest;
    }

    //! Returns the number of items in the queue.
    size_t size() const {
        assert(itemList.size() == itemMap.size());
        return itemList.size();
    }

    //! True if the queue is empty.
    bool empty() const {
        return size() == 0;
    }

private:
    /* Implementation note:
     * The values themselves are stored in order in the itemList, a list of (key, value) pairs.
     * The itemMap associates keys with iterators into the itemList, allowing elements of the list to be found
     * in constant time. When a value is moved to the front of the queue, the list must be updated, but the
     * iterators in the map remain valid.
     */
    std::list<pair_type> itemList;
    std::unordered_map<key_type, typename std::list<pair_type>::iterator> itemMap;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_LRUQUEUE_H */
