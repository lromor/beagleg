#include "../gcode-parser/gcode-parser.h"
#include "../gcode-machine-control.h"
#include "experimental-planner.h"
#include "bezier.h"
#include "experimental/path.h"

#include <array>
#include <list>

// template<typename Primitive>
// struct SpeedProfile : public Primitive {};

using CubicBezierSpeedProfile = CubicBezier<1, float>;

// Defines the final product of this simulation.
// This is meant to be passed to an hardware/software
// step generator.
// This is the last step of the planning pipeline.
// Once we have the digested trajectory, convert each
// trajectory primitive into this struct.
struct TrajectorySegment {
  double requested_feedrate; // Original speed at which this segment
                          // should be traveled.
  CubicBezierSpeedProfile speed;
  CubicBezierPathPrimitive<3> path;
};

// Higher level object to perform planning.
struct PlannedTrajectorySegment {
  UniquePathPrimitive path;
  double requested_feedrate;
  double planned_final_speed;
  double planned_final_accel;
};

inline bool is_euclidean_axis(GCodeParserAxis axis) {
  return axis == AXIS_X || axis == AXIS_Y || axis == AXIS_Z;
}

class ExperimentalPlanner::Impl : public GCodeParser::EventReceiver{
public:
  Impl(MachineControlConfig *config, double tolerance_mm)
    : cfg_(config), tolerance_mm_(tolerance_mm) {}


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
    // Let's immediately convert into absolute step coordinates.
    LinePathPrimitive<3> line;
    // Dirty thing to get first 3 axes.
    for (auto a : AllAxes()) {
      if (!is_euclidean_axis(a)) continue;
      line.end[a] = std::lround(absolute_pos[a] * cfg_->steps_per_mm[a]);
    }
    Enqueue(line, feed_mm_p_sec);
    return true;
  }

  virtual bool rapid_move(float feed_mm_p_sec,
                          const AxesRegister &absolute_pos) {
    float g0_feedrate_mm_per_sec;
    for (const GCodeParserAxis axis : AllAxes()) {
      if (cfg_->max_feedrate[axis] < g0_feedrate_mm_per_sec) {
        g0_feedrate_mm_per_sec = cfg_->max_feedrate[axis];
      }
    }
    return coordinated_move(g0_feedrate_mm_per_sec, absolute_pos);
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

  template<template<size_t> class PathPrimitiveDerived>
  bool Enqueue(const PathPrimitiveDerived<3> &v, const double requested_feedrate) {
    static_assert(std::is_base_of<PathPrimitive<3>, PathPrimitiveDerived<3>>::value,
                  "TrajectoryPrimitiveDerived should inherit from Planner::TrajectoryPrimitive");
    return Enqueue(&v, requested_feedrate);
  }

  // Force execution of already enqueued segments causing the machine
  // to stop when reaching the last.
  void Flush() {}

  bool Init() { return true; }

  bool Enqueue(const PathPrimitive<3> *t, const double requested_feedrate) {
    Plan(t);
    return true;
  }

  void ComputeSpeedProfile() {

  }

  void Plan(const PathPrimitive<3> *new_segment) {
    // // First segment, nothing to do.
    // if (!end_) {
    //   return;
    // }

    // // Let's blend the new and last segment.
    // const std::vector<MotionQueue> = BlendTrajectorySegments(end_.get(), new_segment);

    // // Add the blended segments to the planning_buffer.
    // planning_buffer_.push();

    // // Now recompute the speed profile.
    // ComputeSpeedProfile();
  }

private:
  MachineControlConfig *cfg_;
  // std::list<UniquePlannedTrajectoryPrimitive> planning_buffer_;
  double tolerance_mm_;
};

ExperimentalPlanner *ExperimentalPlanner::Create(MachineControlConfig *config, double tolerance_mm) {
  Impl *impl = new Impl(config, tolerance_mm);
  if (!impl->Init()) {
    delete impl;
    return NULL;
  }
  return new ExperimentalPlanner(impl);
}

GCodeParser::EventReceiver *ExperimentalPlanner::ParseEventReceiver() { return impl_.get(); }

ExperimentalPlanner::ExperimentalPlanner(Impl *impl) : impl_(impl) {}
ExperimentalPlanner::~ExperimentalPlanner() = default;
