#include "Trace.h"

namespace attpc {
namespace common {

Trace::Trace(const HardwareAddress& hwaddr_)
: hwaddr(hwaddr_)
, pad(boost::none)
, data(decltype(data)::Zero(512))
{}

Trace::Trace(const HardwareAddress& hwaddr_, const padid_type pad_)
: hwaddr(hwaddr_)
, pad(pad_)
, data(decltype(data)::Zero(512))
{}

Trace::Trace(const HardwareAddress& hwaddr_, const padid_type pad_, const Eigen::Ref<Eigen::ArrayXi>& data_)
: hwaddr(hwaddr_)
, pad(pad_)
, data(data_)
{}

Trace::Trace(const Trace& other)
: hwaddr(other.hwaddr)
, pad(other.pad)
, data(other.data)
{}

Trace::Trace(Trace&& other)
: hwaddr(std::move(other.hwaddr))
, pad(std::move(other.pad))
, data(std::move(other.data))
{}

Trace& Trace::operator=(const Trace& other) {
    hwaddr = other.hwaddr;
    pad = other.pad;
    data = other.data;
    return *this;
}

Trace& Trace::operator=(Trace&& other) {
    hwaddr = std::move(other.hwaddr);
    pad = std::move(other.pad);
    data = std::move(other.data);
    return *this;
}

bool Trace::operator<(const Trace& other) const {
    return hwaddr < other.hwaddr;
}

}
}
