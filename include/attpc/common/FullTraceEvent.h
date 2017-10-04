#ifndef ATTPC_COMMON_FULLTRACEEVENT_H
#define ATTPC_COMMON_FULLTRACEEVENT_H

#include <map>
#include <iterator>
#include <type_traits>
#include "attpc/common/Trace.h"
#include "attpc/common/HardwareAddress.h"

namespace attpc {
namespace common {

/**
 * @brief Represents a merged event containing full traces.
 *
 * The qualification of a "full-trace" event is made by comparison to a "peaks-only" event that contains only
 * a single data point for each channel. Instances of this class have a full 512-sample array for each channel
 * present in the event.
 *
 * In addition to the Trace data, this class also stores the event ID and event timestamp.
 */
class FullTraceEvent {
public:
    /**
     * @brief An adapter for iterating over mapped values.
     *
     * Typically, the iterators from std::map and std::unordered_map return std::pair objects when they are
     * dereferenced. This is not ideal in our case since the Trace objects contain a copy of the HardwareAddress
     * object that's used as a key, so the std::pair would contain two separate instances of this address. This
     * adapter wraps the standard iterator and returns only the Trace when it is dereferenced.
     */
    template <class BaseIterator, bool isConst>
    class MapIteratorAdapter {
    public:
        // These five typedefs are required by the standard

        //! The iterator category, as defined by the standard (e.g. input, output, etc.)
        using iterator_category = typename std::iterator_traits<BaseIterator>::iterator_category;
        // Explanation of the conditional expression below: If the template parameter isConst is true,
        // then value_type should be the const version (std::add_const) of the mapped type. Otherwise, it should
        // be the non-const version. The type is declared as "[...]value_type::second_type" since we want to
        // return the second element of each std::pair given by the BaseIterator.
        //! The type of the values returned when dereferencing the iterator
        using value_type = typename std::conditional<
            isConst,
            typename std::add_const<typename std::iterator_traits<BaseIterator>::value_type::second_type>::type,
            typename std::iterator_traits<BaseIterator>::value_type::second_type
            >::type;
        //! The type representing the distance between two iterators.
        using difference_type = typename std::iterator_traits<BaseIterator>::difference_type;
        //! The type of a pointer to the type iterated over
        using pointer = value_type*;
        //! The type of a reference to the type iterated over
        using reference = value_type&;

        //! Default constructor
        MapIteratorAdapter() = default;

        /**
         * @brief Construct using a base iterator instance
         * @param baseIter The base iterator to adapt.
         */
        MapIteratorAdapter(BaseIterator&& baseIter) : base(baseIter) {}

        //! Test for equality
        bool operator==(const MapIteratorAdapter& other) { return base == other.base; }
        //! Test for inequality
        bool operator!=(const MapIteratorAdapter& other) { return base != other.base; }

        //! Dereference the iterator
        reference operator*() { return (*base).second; }
        //! Dereference the iterator
        pointer operator->() { return &(base->second); }
        //! Increment the iterator
        MapIteratorAdapter& operator++() {
            ++base;
            return *this;
        }
        //! Increment the iterator
        MapIteratorAdapter operator++(int) {
            MapIteratorAdapter result = *this;
            ++(*this);
            return result;
        }
        //! Decrement the iterator
        MapIteratorAdapter& operator--() {
            --base;
            return *this;
        }
        //! Decrement the iterator
        MapIteratorAdapter operator--(int) {
            MapIteratorAdapter result = *this;
            --(*this);
            return result;
        }

    private:
        BaseIterator base;
    };

    //! An iterator over the Traces in the event
    using iterator = MapIteratorAdapter<std::map<HardwareAddress, Trace>::iterator, false>;
    //! A constant iterator over the Traces in the event
    using const_iterator = MapIteratorAdapter<std::map<HardwareAddress, Trace>::const_iterator, true>;

    /**
     * @brief Default constructor
     *
     * This sets the event ID and timestamp to zero. The event will contain no traces.
     */
    FullTraceEvent();
    /**
     * @brief Construct the event and set the event ID and timestamp to the given values.
     *
     * The event will contain no traces.
     *
     * @param eventId_   The event ID
     * @param timestamp_ The timestamp
     */
    FullTraceEvent(const evtid_type eventId_, const timestamp_type timestamp_);

    /**
     * @brief Insert a Trace into the event.
     *
     * This has the same behavior as the underlying standard library container's emplace function. If the event
     * already contains a Trace with the same HardwareAddress, then no insertion will take place, and an iterator
     * to the existing Trace will be returned along with a false boolean indicating that no insertion happened.
     * Otherwise, an iterator to the newly inserted Trace will be returned along with a true boolean indicating
     * that the insertion did occur.
     *
     * @param trace The Trace to (potentially) insert.
     * @returns     A tuple containing an iterator and a bool. Their meanings are described above.
     */
    std::tuple<iterator, bool> insertTrace(const Trace& trace);
    //! @overload
    std::tuple<iterator, bool> insertTrace(Trace&& trace);

    /**
     * @brief Insert the given Trace, replacing any existing Trace with the same HardwareAddress.
     * @param  trace The trace to insert.
     * @return       An iterator to the inserted Trace.
     */
    iterator insertOrReplaceTrace(const Trace& trace);
    //! @overload
    iterator insertOrReplaceTrace(Trace&& trace);

    /**
     * @brief Find the Trace with the given HardwareAddress in this event.
     *
     * If the Trace is not found, end() is returned.
     *
     * @param  hwaddr The HardwareAddress to find.
     * @return        An iterator to the found trace, or end().
     */
    iterator findTrace(const HardwareAddress& hwaddr);
    //! @overload
    const_iterator findTrace(const HardwareAddress& hwaddr) const;

    //! Get an iterator to the first Trace in the event.
    iterator begin() noexcept { return traces.begin(); }
    //! Get a constant iterator to the first trace in the event.
    const_iterator begin() const noexcept { return traces.begin(); }
    //! Get a constant iterator to the first trace in the event.
    const_iterator cbegin() const noexcept { return traces.cbegin(); }

    //! Get an iterator to the past-the-end trace in the event.
    iterator end() noexcept { return traces.end(); }
    //! Get a constant iterator to the first trace in the event.
    const_iterator end() const noexcept { return traces.end(); }
    //! Get a constant iterator to the past-the-end trace in the event.
    const_iterator cend() const noexcept { return traces.cend(); }

    //! Get the number of traces in the event
    size_t numTraces() const { return traces.size(); }

    //! Get the event ID
    evtid_type getEventId() const { return eventId; }
    //! Set the event ID
    void setEventId(const evtid_type newEventId) { eventId = newEventId; }

    //! Get the event timestamp
    timestamp_type getTimestamp() const { return timestamp; }
    //! Set the event timestamp
    void setTimestamp(const timestamp_type newTimestamp) { timestamp = newTimestamp; }

    /**
     * @brief Compare two events for equality.
     *
     * Two events are equal if:
     * - Their event IDs are equal
     * - Their timestamps are equal
     * - All traces are equal, and neither contains any traces not found in the other
     *
     * @return True if the conditions above are met
     */
    bool operator==(const FullTraceEvent& other) const;

private:
    evtid_type eventId;
    timestamp_type timestamp;
    std::map<HardwareAddress, Trace> traces;
};

}
}

#endif /* end of include guard: ATTPC_COMMON_FULLTRACEEVENT_H */
