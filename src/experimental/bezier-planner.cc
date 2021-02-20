#include "../gcode-machine-control.h"

#include "bezier-planner.h"
#include "experimental/trajectory.h"
#include <array>
#include <list>

template<typename Primitive>
struct SpeedProfile : public Primitive {};

using CubicBezierSpeedProfile = SpeedProfile<CubicBezier<1>>;

// Defines the final product of this simulation.
// This is meant to be passed to an hardware/software
// step generator.
// This is the last step of the planning pipeline.
// Once we have the digested trajectory, convert each
// trajectory primitive into this struct.
struct MotionSegment {
  // Tag to the original trajectory
  size_t id; // Reference to the original command.
  double requested_feedrate; // Original speed at which this segment
                          // should be traveled.
  CubicBezierSpeedProfile speed;
  CubicBezierTrajectoryPrimitive<3> trajectory;
};

struct PlannedTrajectoryArc {
  size_t id;
  std::unique_ptr<TrajectoryPrimitive<3>> curve;
  double requested_feedrate;
  double planned_final_speed;
  double planned_final_accel;
};

class Planner::Impl {
public:

  Impl(MachineControlConfig *config, double tolerance_mm)
    : config_(config), tolerance_mm_(tolerance_mm) {}

  bool Init() { return true; }
  bool Enqueue(const TrajectoryPrimitive<3> *t) {
    Plan(t);
    return true;
  }

  void ComputeSpeedProfile() {

  }

  void Plan(const TrajectoryPrimitive<3> *new_segment) {
    // First segment, nothing to do.
    if (!end_) {
      return;
    }

    // Let's blend the new and last segment.
    const std::vector<MotionQueue> = BlendTrajectorySegments(end_.get(), new_segment);

    // Add the blended segments to the planning_buffer.
    planning_buffer_.push();

    // Now recompute the speed profile.
    ComputeSpeedProfile();
  }

private:
  MachineControlConfig *config_;
  typedef std::unique_ptr<TrajectoryPrimitive<3>> UniqueTrajectoryPrimitive;

  std::list<UniquePlannedTrajectoryPrimitive> planning_buffer_;

  double tolerance_mm_;
};

Planner *Planner::Create(MachineControlConfig *config, double tolerance_mm) {
  Impl *impl = new Impl(config, tolerance_mm);
  if (!impl->Init()) {
    delete impl;
    return NULL;
  }
  return new Planner(impl);
}

Planner::Planner(Impl *impl) : impl_(impl) {}
Planner::~Planner() = default;

bool Planner::Enqueue(const TrajectoryPrimitive<3> *t) {
  return impl_->Enqueue(t);
}
