#include "GRAWHeader.h"

namespace attpc {
namespace mergers {

GRAWHeader::GRAWHeader(const RawFrame& rawFrame)
: metaType(rawFrame)
, frameSize(rawFrame)
, dataSource(rawFrame)
, frameType(rawFrame)
, revision(rawFrame)
, headerSize(rawFrame)
, itemSize(rawFrame)
, itemCount(rawFrame)
, eventTime(rawFrame)
, eventIdx(rawFrame)
, coboIdx(rawFrame)
, asadIdx(rawFrame)
, readOffset(rawFrame)
, status(rawFrame)
, hitPat_0(rawFrame)
, hitPat_1(rawFrame)
, hitPat_2(rawFrame)
, hitPat_3(rawFrame)
, multip_0(rawFrame)
, multip_1(rawFrame)
, multip_2(rawFrame)
, multip_3(rawFrame)
, windowOut(rawFrame)
{}

}
}
