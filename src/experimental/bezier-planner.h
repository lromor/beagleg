#ifndef __BEZIER_PLANNER_H_
#define __BEZIER_PLANNER_H_
#include <queue>
#include "trajectory.h"
#include "bezier.h"
#include "../gcode-machine-control.h"

class Planner {
public:
  ~Planner();
  static Planner *Create(MachineControlConfig *config, double tolerance_mm = 0.1f);

  template<template<size_t> class TrajectoryPrimitiveDerived>
  bool Enqueue(const TrajectoryPrimitiveDerived<3> &v) {
    static_assert(std::is_base_of<TrajectoryPrimitive<3>, TrajectoryPrimitiveDerived<3>>::value,
                  "TrajectoryPrimitiveDerived should inherit from Planner::TrajectoryPrimitive");
    return Enqueue(&v);
  }


  // Force execution of already enqueued segments causing the machine
  // to stop when reaching the last.
  void Flush() {}

private:
  bool Enqueue(const TrajectoryPrimitive<3> *t);

  class Impl;
  Planner(Impl *impl);
  std::unique_ptr<Impl> impl_;
};


#endif // __BEZIER_PLANNER_H_
