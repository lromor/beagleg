// In this program, we try to simulate
// a 3d trajectory with the new planner.
// The is just a proof of concept, just a specific
// and simple example for a 3d trajectory, with fixed and uniform
// motor and trajectory tolerances. No tooling is simulated
// as we are focusing on the pure kinematics of the problem.
#ifndef __EXPERIMENTAL_PLANNER_H_
#define __EXPERIMENTAL_PLANNER_H_
#include <array>
#include <cmath>
#include <iterator>
#include <list>
#include <math.h>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "../gcode-machine-control.h"
#include "../config-parser.h"

#include "bezier.h"
#include "vector.h"

using FloatVector3D = Vector<float, 3>;
using IntVector3D = Vector<int, 3>;
using PositionSteps = IntVector3D;

// Basic trajectory arc types.
enum class PathPrimitiveType { Line, CubicBezier, CircularArc };

// Define an abstract piece of trajectory.
// The parameter template N is the dimensionality of the trajectory.
struct PathPrimitive {
  using Type = PathPrimitiveType;
  PositionSteps end;
  float speed;
  virtual const double ComputeArcLength(const PositionSteps &start) const = 0;
  virtual const Type GetType() const = 0;
};

struct CircularArcPathPrimitive : public PathPrimitive {
  constexpr static PathPrimitiveType type = PathPrimitiveType::CircularArc;
  // Center position relative to the start.
  FloatVector3D center;
  virtual const double ComputeArcLength(const PositionSteps &start) const final {
    const double chord_length = (this->end - start).norm();
    const double ray_length = center.norm();
    return ray_length * 2 * std::asin(ray_length / chord_length);
  }
  virtual const PathPrimitiveType GetType() const final { return type; }
};

struct LinePathPrimitive : public PathPrimitive {
  constexpr static PathPrimitiveType type = PathPrimitiveType::Line;
  virtual const double ComputeArcLength(const PositionSteps &start) const final {
    return (this->end - start).norm();
  }
  virtual const PathPrimitiveType GetType() const final { return type; }
};

struct CubicBezierPathPrimitive : public PathPrimitive {
  constexpr static PathPrimitiveType type = PathPrimitiveType::CubicBezier;
  // cp0 and cp3 are the previous point, and the end.
  FloatVector3D cp1, cp2;
  virtual const double ComputeArcLength(const PositionSteps &start) const final {
    FloatVector3D float_start, float_end;
    std::copy(start.begin(), start.end(), float_start.begin());
    std::copy(end.begin(), end.end(), float_end.begin());
    const auto b = CubicBezier<3, float>{float_start, cp1, cp2, float_end};
    return b.length();
  }
  virtual const PathPrimitiveType GetType() const final { return type; }
};

typedef std::unique_ptr<PathPrimitive> UniquePathPrimitive;
using CubicBezierSpeedProfile = CubicBezier<1, float>;

// Defines the final product of this simulation.
// This is meant to be passed to an hardware/software
// step generator.
// This is the last step of the planning pipeline.
// Once we have the digested trajectory, convert each
// trajectory primitive into this struct.
// Each GCodeRequestedTrajectorySegment can have multiple
// of these objects, depending on requirements.
struct TrajectorySegment {
  float requested_feedrate;
  CubicBezierSpeedProfile speed;
  CubicBezierPathPrimitive path;
  float planned_final_speed;
  float planned_final_accel;
};

// Higher level object to perform planning.
// Each of these trajectory pieces can be split to smaller units
// based on needs. There's a one-to-one correspondence between
// a GCode command and this object instance.
// TODO: Add some sort of tagging to backtrack currently
// executing gcode line.
struct GCodeRequestedTrajectorySegment {
  GCodeRequestedTrajectorySegment(
    UniquePathPrimitive &segment_, const PositionSteps &start,
    double requested_speed)
    : segment(std::move(segment_)),
      delta_steps(segment->end - start),
      requested_speed_mm_per_sec(requested_speed),
      arc_length_mm(segment->ComputeArcLength(start)) {}
  // Path representation
  UniquePathPrimitive segment;

  // Derived values
  const IntVector3D delta_steps; // Difference to previous position.
  const double requested_speed_mm_per_sec;
  const double arc_length_mm;
};

class ExperimentalPlanner {
public:
  ExperimentalPlanner(MachineControlConfig *config, float tolerance_mm = 0.1f)
    : config_(config), tolerance_mm_(tolerance_mm) {}

  template<class PathPrimitiveDerived>
  bool Enqueue(const PathPrimitiveDerived &path, const double requested_feedrate);

private:
  MachineControlConfig *config_;
  const float tolerance_mm_;

  // // Force execution of already enqueued segments causing the machine
  // // to stop when reaching the last.
  // void Flush() {}

  // bool Init() { return true; }

  // bool Enqueue(const PathPrimitive *t, const double requested_feedrate) {
  //   //Plan(t);
  //   return true;
  // }

  // void ComputeSpeedProfile() {

  // }

  // void Plan(const PathPrimitive *new_segment) {
  //   // // First segment, nothing to do.
  //   // if (!end_) {
  //   //   return;
  //   // }

  //   // // Let's blend the new and last segment.
  //   // const std::vector<MotionQueue> = BlendTrajectorySegments(end_.get(), new_segment);

  //   // // Add the blended segments to the planning_buffer.
  //   // planning_buffer_.push();

  //   // // Now recompute the speed profile.
  //   // ComputeSpeedProfile();
  // }

  std::vector<GCodeRequestedTrajectorySegment> planning_buffer_;
};

#endif // __EXPERIMENTAL_PLANNER_H_
