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
#include "bezier-planner.h"
#include "vector.h"

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
    line.end = Vector<3>{{absolute_pos[AXIS_X], absolute_pos[AXIS_Y], absolute_pos[AXIS_Z]}};
    line.requested_feedrate = feed_mm_p_sec;
    planner_->Enqueue(line);
    return true;
  }
  virtual bool rapid_move(float feed_mm_p_sec,
                          const AxesRegister &absolute_pos) {
    LineTrajectoryPrimitive<3> line;
    line.end = Vector<3>{{absolute_pos[AXIS_X], absolute_pos[AXIS_Y], absolute_pos[AXIS_Z]}};
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
