#include "attpc/common/Trace.h"

namespace attpc {
namespace common {

Trace::Trace(const HardwareAddress& hwaddr_)
: hwaddr(hwaddr_)
, pad(boost::none)
, data(decltype(data)::Zero(512))
{}

Trace::Trace(const HardwareAddress& hwaddr_, const boost::optional<padid_type> pad_)
: hwaddr(hwaddr_)
, pad(pad_)
, data(decltype(data)::Zero(512))
{}

Trace::Trace(const HardwareAddress& hwaddr_, const boost::optional<padid_type> pad_, const Eigen::Ref<const ArrayType>& data_)
: hwaddr(hwaddr_)
, pad(pad_)
, data(data_)
{}

bool Trace::operator<(const Trace& other) const {
    return hwaddr < other.hwaddr;
}

bool Trace::operator==(const Trace& other) const {
    return (hwaddr == other.hwaddr) && (data == other.data).all() && (pad == other.pad);
}

}
}
