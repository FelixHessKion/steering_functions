#pragma once

#include "CCStateSpace.hpp"

using namespace ompl::base;

namespace cc_hc_steer_functions {
class DubinsCCStateSpace : public CCStateSpace {
  DubinsCCStateSpace() {
    setName("Dubins_CC_" + getName());
    type_ = STATE_SPACE_TYPE_COUNT + 1;
    addSubspace(std::make_shared<RealVectorStateSpace>(2), 1.0);
    addSubspace(std::make_shared<SO2StateSpace>(), 0.5);
    lock();
  }

  ~DubinsCCStateSpace() override = default;

  double distance(const State *state1, const State *state2) const override {}

  void setBounds(const RealVectorBounds &bounds) {
    as<RealVectorStateSpace>(0)->setBounds(bounds);
  }

  const RealVectorBounds &getBounds() const {
    return as<RealVectorStateSpace>(0)->getBounds();
  }
};
}  // namespace cc_hc_steer_functions
