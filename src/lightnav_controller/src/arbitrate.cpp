#include "lightnav_controller/arbitrate.hpp"

namespace lightnav_controller {

namespace {

ArbitrationResult makeStop(StopReason reason) {
  ArbitrationResult result;
  result.reason = reason;
  return result;
}

ArbitrationResult makeAdopted(const LayerOutput& output, Layer layer) {
  ArbitrationResult result;
  result.twist = output.twist;
  result.is_hard_stop = false;
  result.source_layer = layer;
  result.reason = StopReason::kOk;
  return result;
}

}  // namespace

ArbitrationResult arbitrate(const ArbitrationInput& input) {
  if (!input.is_pose_fresh) {
    return makeStop(StopReason::kPoseStale);
  }
  if (input.avoid.is_asserted && input.is_grid_fresh) {
    if (input.avoid.reason != StopReason::kOk) {
      // Avoid layer is waiting (no feasible arc / stuck): hold still and say why.
      ArbitrationResult waiting = makeStop(input.avoid.reason);
      waiting.source_layer = Layer::kAvoid;
      return waiting;
    }
    return makeAdopted(input.avoid, Layer::kAvoid);
  }
  if (input.follow.is_asserted) {
    return makeAdopted(input.follow, Layer::kFollow);
  }
  // follow.reason is kOk only when the layer gave no reason; report NO_PATH.
  const StopReason reason =
      input.follow.reason == StopReason::kOk ? StopReason::kNoPath : input.follow.reason;
  return makeStop(reason);
}

}  // namespace lightnav_controller
