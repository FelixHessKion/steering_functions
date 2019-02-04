#pragma once

#include <ompl/base/spaces/SE2StateSpace.h>

using namespace ompl::base;

class CCStateSpace : public CompoundStateSpace {
 public:
  class StateType : public CompoundStateSpace::StateType {
   public:
    StateType() = default;

    double getX() const {
      return as<RealVectorStateSpace::StateType>(0)->values[0];
    }

    double getY() const {
      return as<RealVectorStateSpace::StateType>(0)->values[1];
    }

    double getYaw() const { return as<SO2StateSpace::StateType>(1)->value; }

    void setX(double x) {
      as<RealVectorStateSpace::StateType>(0)->values[0] = x;
    }

    void setY(double y) {
      as<RealVectorStateSpace::StateType>(0)->values[1] = y;
    }

    void setXY(double x, double y) {
      setX(x);
      setY(y);
    }

    void setYaw(double yaw) { as<SO2StateSpace::StateType>(1)->value = yaw; }
  };

  DubinsCCStateSpace() {
    setName("CC_" + getName());
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

  State *allocState() const override;
  void freeState(State *state) const override;

  void registerProjections() override;
};
