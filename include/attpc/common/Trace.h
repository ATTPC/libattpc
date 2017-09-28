#ifndef ATTPC_COMMON_TRACE_H
#define ATTPC_COMMON_TRACE_H

#include <boost/optional.hpp>
#include <Eigen/Core>
#include "attpc/common/types.h"
#include "attpc/common/HardwareAddress.h"

namespace attpc {
namespace common {

class Trace {
public:
    using ScalarType = int16_t;
    using ArrayType = Eigen::Array<ScalarType, Eigen::Dynamic, 1>;

    Trace(const HardwareAddress& hwaddr_);
    Trace(const HardwareAddress& hwaddr_, const boost::optional<padid_type> pad_);
    Trace(const HardwareAddress& hwaddr_, const boost::optional<padid_type> pad_, const Eigen::Ref<const ArrayType>& data_);

    inline ScalarType& operator()(const Eigen::Index idx) { return data(idx); }
    inline const ScalarType& operator()(const Eigen::Index idx) const { return data(idx); }

    bool operator<(const Trace& other) const;
    bool operator==(const Trace& other) const;

    const HardwareAddress& getHardwareAddress() const { return hwaddr; }
    const boost::optional<padid_type>& getPad() const { return pad; }
    const ArrayType& getData() const { return data; }

private:
    HardwareAddress hwaddr;
    boost::optional<padid_type> pad;
    ArrayType data;
};

}
}

#endif /* end of include guard: ATTPC_COMMON_TRACE_H */
