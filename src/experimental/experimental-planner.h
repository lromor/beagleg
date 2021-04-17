// In this program, we try to simulate
// a 3d trajectory with the new planner.
// The is just a proof of concept, just a specific
// and simple example for a 3d trajectory, with fixed and uniform
// motor and trajectory tolerances. No tooling is simulated
// as we are focusing on the pure kinematics of the problem.
#ifndef __PLANNER_EXPERIMENT_H_
#define __PLANNER_EXPERIMENT_H_
#include <bits/stdint-uintn.h>
#include <cstdint>
#include <iostream>
#include <queue>
#include <cassert>
#include <unistd.h>
#include <memory>
#include <sstream>

#include "../gcode-parser/gcode-parser.h"
#include "../gcode-machine-control.h"
#include "../config-parser.h"
#include "path.h"
#include "vector.h"

class ExperimentalPlanner {
public:
  static ExperimentalPlanner *Create(MachineControlConfig *config, double tolerance_mm = 0.1f);
  virtual ~ExperimentalPlanner();

  // Return the receiver for parse events. The caller must not assume ownership
  // of the returned pointer.
  GCodeParser::EventReceiver *ParseEventReceiver();

private:
  class Impl;
  ExperimentalPlanner(Impl *impl);
  std::unique_ptr<Impl> impl_;
};
#endif // __PLANNER_EXPERIMENT_H_
