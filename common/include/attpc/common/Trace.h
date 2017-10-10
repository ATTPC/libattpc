#ifndef ATTPC_COMMON_TRACE_H
#define ATTPC_COMMON_TRACE_H

#include <boost/optional.hpp>
#include <Eigen/Core>
#include "attpc/common/types.h"
#include "attpc/common/HardwareAddress.h"

namespace attpc {
namespace common {

/**
 * @brief Represents the data recorded by one channel in the GET electronics.
 *
 * This is essentially a fancy array. In addition to storing the 512 samples from the data file, it also
 * stores the HardwareAddress and pad number corresponding to the channel.
 *
 * The pad number is stored using a boost::optional object.
 */
class Trace {
public:
    using ScalarType = int16_t;  //! The scalar data type stored in the array
    using ArrayType = Eigen::Array<ScalarType, Eigen::Dynamic, 1>;  //! The type of the array used for data storage

    /**
     * @brief Construct a Trace
     *
     * In this version, the HardwareAddress is set, but the pad number is left as boost::none. The data array elements
     * are initialized to zero.
     *
     * @param hwaddr_ The HardwareAddress of the channel.
     */
    Trace(const HardwareAddress& hwaddr_);
    /**
     * @brief Construct a Trace
     *
     * In this version, the HardwareAddress and pad number are initialized to the given values. The data array elements
     * are initialized to zero.
     *
     * @param hwaddr_ The HardwareAddress of the channel.
     * @param pad_    The pad number. (May also be boost::none)
     */
    Trace(const HardwareAddress& hwaddr_, const boost::optional<padid_type> pad_);
    /**
     * @brief Construct a Trace
     *
     * In this version, the HardwareAddress and pad number are initialized to the given values, and the data array
     * elements are set to the values in the provided array.
     *
     * @param hwaddr_ The HardwareAddress of the channel.
     * @param pad_    The pad number. (May also be boost::none)
     * @param data_   The data array elements.
     */
    Trace(const HardwareAddress& hwaddr_, const boost::optional<padid_type> pad_, const Eigen::Ref<const ArrayType>& data_);

    /**
     * @brief Access an element of the data array by index.
     *
     * Bounds checking is not performed explicitly beyond that which is done by Eigen in a debug build.
     *
     * @param  idx The index.
     * @return     The value stored at that index.
     */
    inline ScalarType& operator()(const Eigen::Index idx) { return data(idx); }
    //! @overload
    inline const ScalarType& operator()(const Eigen::Index idx) const { return data(idx); }

    /**
     * @brief Determine if one Trace is less than another one.
     *
     * The ordering is done by HardwareAddress. See that class's documentation for more information.
     *
     * @return True if the left-hand side is less than the right-hand side.
     */
    bool operator<(const Trace& other) const;
    /**
     * @brief Compare two Traces for equality.
     *
     * Two traces are said to be equal if:
     * - The HardwareAddress instances are equal
     * - The pad numbers are equal
     * - All data array elements are equal
     *
     * @return True if the conditions above are met.
     */
    bool operator==(const Trace& other) const;


    //! Get a reference to this Trace's HardwareAddress.
    const HardwareAddress& getHardwareAddress() const { return hwaddr; }

    //! Get a reference to this Trace's pad number. (May be boost::none)
    const boost::optional<padid_type>& getPad() const { return pad; }

    //! Set the pad number for this Trace
    void setPad(const boost::optional<padid_type>& value) { pad = value; }

    //! Get a const reference to the data array.
    const ArrayType& getData() const { return data; }

private:
    HardwareAddress hwaddr;
    boost::optional<padid_type> pad;
    ArrayType data;
};

}
}

#endif /* end of include guard: ATTPC_COMMON_TRACE_H */
