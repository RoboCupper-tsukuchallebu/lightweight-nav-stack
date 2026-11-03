#pragma once

#include <optional>
#include <string>
#include <vector>

namespace lightnav
{

// Names of the stale causes of one input source. Empty when the input is usable.
//  - "<sourceName>": no data yet, or the last data is older than timeoutSeconds
//  - "tf:<sensorFrame>": the last transform lookup failed
std::vector<std::string> findStaleSources(
  const std::string & sourceName, const std::string & sensorFrame,
  const std::optional<double> & lastDataSeconds, bool hasLastTransformFailed,
  double nowSeconds, double timeoutSeconds);

}  // namespace lightnav
