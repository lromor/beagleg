#include "planner.h"

#include <algorithm>
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
#include "vector.h"
#include "experimental-planner.h"

static inline bool is_euclidean_axis(GCodeParserAxis axis) {
  return axis == AXIS_X || axis == AXIS_Y || axis == AXIS_Z;
}

static inline float compute_g0_feedrate_mm_per_sec(const AxesRegister &max_feedrate) {
  return *std::max_element(max_feedrate.begin(), max_feedrate.end());
}

class GCodeEventReceiver : public GCodeParser::EventReceiver {
public:
  GCodeEventReceiver(MachineControlConfig *config)
    : cfg_(config), g0_feedrate_mm_per_sec_(compute_g0_feedrate_mm_per_sec(config->max_feedrate)),
      planner_(new ExperimentalPlanner(config)) {
  }

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

  virtual bool coordinated_move(float feed_mm_p_sec,
                                const AxesRegister &absolute_pos) {
    // Let's immediately convert into absolute step coordinates.
    LinePathPrimitive line;
    // We assume the first 3 axes are always the euclidean ones.
    // For now we don't really care about the others.
    for (auto a : AllAxes()) {
      if (!is_euclidean_axis(a)) break;
      line.end[a] = std::lround(absolute_pos[a] * cfg_->steps_per_mm[a]);
    }
    // We need
    planner_->Enqueue(line, feed_mm_p_sec);
    return true;
  }

  virtual bool rapid_move(float feed_mm_p_sec,
                          const AxesRegister &absolute_pos) {
    // We use the maximum speed across all the axes. It will be planner's
    // duty to adapt the maximum feedrate depending on the speed vector direction.
    return coordinated_move(g0_feedrate_mm_per_sec_, absolute_pos);
  }

  virtual bool arc_move(float feed_mm_p_sec,
                        GCodeParserAxis normal_axis, bool clockwise,
                        const AxesRegister &start,
                        const AxesRegister &center,
                        const AxesRegister &end) {
    return true;
  }

  // G5, G5.1
  // Move in a cubic spine from absolute "start" to "end" given the absolute
  // control points "cp1" and "cp2".
  // The default implementation linearlizes curve and calls coordinated_move()
  // with the segments.
  virtual bool spline_move(float feed_mm_p_sec,
                           const AxesRegister &start,
                           const AxesRegister &cp1, const AxesRegister &cp2,
                           const AxesRegister &end) {
    return true;
  }

  virtual const char *unprocessed(char letter, float value,
                                  const char *rest_of_line) {
    return NULL;
  }

private:
  MachineControlConfig *cfg_;
  float g0_feedrate_mm_per_sec_;
  std::unique_ptr<ExperimentalPlanner> planner_;
};

int main(int argc, char *argv[]) {
  GCodeParser::Config parser_cfg;
  MachineControlConfig config;
  ConfigParser config_parser;

  config_parser.SetContentFromFile("../../sample.config");
  config.ConfigureFromFile(&config_parser);

  config.require_homing = false;
  config.range_check = false;

  GCodeEventReceiver event_parser = GCodeEventReceiver(&config);
  GCodeParser parser(parser_cfg, &event_parser);
  parser.ReadFile(stdin, stderr);

  return 0;
}
