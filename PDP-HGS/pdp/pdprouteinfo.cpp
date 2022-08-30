#include "pdprouteinfo.h"

#include "utils/application.h"

namespace pdp {

PDPRouteInfo::PDPRouteInfo() : distance(0), begin(0), end(0), size(0), valid(true) {
}

const PDPRouteInfo& PDPRouteInfo::operator=(const PDPRouteInfo& c) {
  std::memcpy(this, &c, sizeof(*this));
  return *this;
}

PDPRouteInfo::PDPRouteInfo(int singleItem) : begin(singleItem), end(singleItem), size(1), valid(true) {
  distance = 0;
}
};  // namespace pdp
