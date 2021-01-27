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
#include "trajectory.h"
#include "bezier.h"

template<typename Primitive>
struct SpeedProfile : public Primitive {};

using CubicBezierSpeedProfile = SpeedProfile<CubicBezier<1>>;

// Defines the final product of this simulation.
// This is meant to be passed to an hardware/software
// step generator.
struct MotionSegment {
  // Tag to the original trajectory
  size_t id; // Reference to the original command.
  CubicBezierSpeedProfile speed;
  CubicBezierTrajectoryPrimitive<3> trajectory;
};

class Planner {
public:

  Planner(MachineControlConfig *config, double tolerance_mm = 0.1f)
    : config_(config), trajectory_(), min_time_lookahead_(1),
      tolerance_mm_(tolerance_mm),
      last_speed_(),
      last_accel_(),
      last_jerk_()
      {}

  struct QueueElement {
    std::unique_ptr<TrajectoryPrimitive<3>> curve;
    TrajectoryPrimitiveType type;
  };

  template<typename TrajectoryPrimitiveClass>
  bool Enqueue(const TrajectoryPrimitiveClass &v) {
    trajectory_.push({std::unique_ptr<TrajectoryPrimitive<3>>(new TrajectoryPrimitiveClass(v)), TrajectoryPrimitiveClass::type});
    Plan();
    return true;
  }

  void Plan() {

    // Compute new segment length

    // Pipeline

    // 1st stage, split into small chunks of fixed maximum amount of steps
    // starting from the end, compute the most optimal previous speed and smooth out
    // corners using bezier curves
    // if after looahead_size - 1, we have enough space to decelerate from the planned speed
    // to zero we proceed with sending the step to the fpga.
    // Otherwise we wait for more trajectory elements
  }

private:
  MachineControlConfig *config_;
  std::queue<TrajectorySegment> trajectory_;
  std::queue<MotionSegment> planning_buffer_;
  size_t min_time_lookahead_; // Since our last stop, before pushing stuff in the backend,
                              // we require to hold at least this amount of motion time in the
                              // planning buffer before starting sending stuff to the motion backend.
  double tolerance_mm_;
  Vector<3> last_pos_;
  float last_speed_;
  float last_accel_;
  float last_jerk_;
};


class GCodeEventReceiver : public GCodeParser::EventReceiver {
public:
  GCodeEventReceiver(Planner *planner) : planner_(planner) {}
  virtual ~GCodeEventReceiver() {}
  // Start program. Use for initialization. Informs about gcode parser so that
  // it is possible to call ParsePair().
  virtual void gcode_start(GCodeParser *parser) {}

  virtual void go_home(AxisBitmap_t axis_bitmap) {}

  virtual void set_speed_factor(float factor) {}
  virtual void set_fanspeed(float value) {}
  virtual void set_temperature(float degrees_c) {}
  virtual void wait_temperature() {}
  virtual void dwell(float time_ms) {}
  virtual void motors_enable(bool enable) {}

  // G1 (coordinated move) and G0 (rapid move). Move to absolute coordinates.
  // First parameter is feedrate in mm/sec if provided, or -1 otherwise.
  //   (typically, the user would need to remember the positive values).
  // The second parameter is an array of absolute coordinates (already
  // preprocessed to be in millimeter), indexed by GCodeParserAxis.
  // Returns true if the move was successful or false if the machine could
  // not move (e.g. because it was beyond limits or other condition).
  virtual bool coordinated_move(float feed_mm_p_sec,
                                const AxesRegister &absolute_pos) {
    LineTrajectoryPrimitive<3> line;
    line.end = Vector<3>{absolute_pos[AXIS_X], absolute_pos[AXIS_Y], absolute_pos[AXIS_Z]};
    line.requested_feedrate = feed_mm_p_sec;
    planner_->Enqueue(line);
    return true;
  }
  virtual bool rapid_move(float feed_mm_p_sec,
                          const AxesRegister &absolute_pos) {
    LineTrajectoryPrimitive<3> line;
    line.end = Vector<3>{absolute_pos[AXIS_X], absolute_pos[AXIS_Y], absolute_pos[AXIS_Z]};
    line.requested_feedrate = feed_mm_p_sec;
    planner_->Enqueue(line);
    return true;
  }

  virtual const char *unprocessed(char letter, float value,
                                  const char *rest_of_line) {
    return NULL;
  }

private:
  Planner *planner_;
};


#endif // __PLANNER_EXPERIMENT_H_
