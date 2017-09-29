#ifndef ATTPC_MERGERS_LRUQUEUE_H
#define ATTPC_MERGERS_LRUQUEUE_H

#include <list>
#include <unordered_map>
#include <utility>
#include <boost/optional.hpp>
#include <cassert>

namespace attpc {
namespace mergers {

template <class Key, class T>
class LRUQueue
{
public:
    using key_type = Key;
    using value_type = T;
    using pair_type = std::pair<key_type, value_type>;

    value_type& insert(const key_type& key, value_type&& value) {
        itemList.emplace_front(key, std::forward<value_type>(value));
        itemMap.emplace(key, itemList.begin());
        return itemList.front().second;
    }

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

    boost::optional<value_type> extract(const key_type& key) {
        auto mapIter = itemMap.find(key);
        if (mapIter == itemMap.end()) {
            return boost::none;
        }

        const key_type foundKey = mapIter->first;
        assert(foundKey == key);
        typename std::list<pair_type>::iterator listIter = mapIter->second;

        value_type item = std::move(listIter.second);
        itemList.erase(listIter);
        itemMap.erase(key);
        return item;
    }

    pair_type extractOldest() {
        pair_type oldest = itemList.back();
        itemList.pop_back();

        const key_type key = oldest.first;
        itemMap.erase(key);

        return oldest;
    }

    size_t size() const {
        assert(itemList.size() == itemMap.size());
        return itemList.size();
    }

    bool empty() const {
        return size() == 0;
    }

private:
    std::list<pair_type> itemList;
    std::unordered_map<key_type, typename std::list<pair_type>::iterator> itemMap;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_LRUQUEUE_H */
