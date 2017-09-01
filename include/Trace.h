#ifndef ATTPC_COMMON_TRACE_H
#define ATTPC_COMMON_TRACE_H

#include <boost/optional.hpp>
#include <Eigen/Core>
#include "types.h"
#include "HardwareAddress.h"

namespace attpc {
namespace common {

class Trace {
public:
    Trace(const HardwareAddress& hwaddr_);
    Trace(const HardwareAddress& hwaddr_, const padid_type pad_);
    Trace(const HardwareAddress& hwaddr_, const padid_type pad_, const Eigen::Ref<Eigen::ArrayXi>& data_);

    inline Eigen::ArrayXi::Scalar& operator()(const Eigen::Index idx) { return data(idx); }
    inline const Eigen::ArrayXi::Scalar& operator()(const Eigen::Index idx) const { return data(idx); }

    bool operator<(const Trace& other) const;

    const HardwareAddress& getHardwareAddress() const { return hwaddr; }
    const boost::optional<padid_type>& getPad() const { return pad; }
    const Eigen::ArrayXi& getData() const { return data; }

private:
    HardwareAddress hwaddr;
    boost::optional<padid_type> pad;
    Eigen::ArrayXi data;
};

}
}

#endif /* end of include guard: ATTPC_COMMON_TRACE_H */
