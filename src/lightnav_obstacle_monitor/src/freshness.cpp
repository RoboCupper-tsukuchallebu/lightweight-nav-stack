#include "lightnav_obstacle_monitor/freshness.hpp"

namespace lightnav
{

std::vector<std::string> findStaleSources(
  const std::string & sourceName, const std::string & sensorFrame,
  const std::optional<double> & lastDataSeconds, bool hasLastTransformFailed,
  double nowSeconds, double timeoutSeconds)
{
  std::vector<std::string> staleSources;
  if (hasLastTransformFailed) {
    staleSources.push_back("tf:" + sensorFrame);
  }
  if (!lastDataSeconds || nowSeconds - *lastDataSeconds > timeoutSeconds) {
    staleSources.push_back(sourceName);
  }
  return staleSources;
}

}  // namespace lightnav
