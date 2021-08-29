/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013, 2016 Henner Zeller <h.zeller@acm.org>
 *
 * This file is part of BeagleG. http://github.com/hzeller/beagleg
 *
 * BeagleG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BeagleG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BeagleG.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "planner.h"

#include <stdlib.h>

#include <cmath>  // We use these functions as they work type-agnostic

#include "common/container.h"
#include "common/logging.h"
#include "gcode-machine-control.h"
#include "gcode-parser/gcode-parser.h"
#include "hardware-mapping.h"
#include "segment-queue.h"

namespace {
// The target position vector is essentially a position in the
// GCODE_NUM_AXES-dimensional space.
//
// An AxisTarget has a position vector, in absolute machine coordinates, and a
// speed when arriving at that position.
//
// The speed is initially the aimed goal; if it cannot be reached, the
// AxisTarget will be modified to contain the actually reachable value. That is
// used in planning along the path.
struct AxisTarget {
  int position_steps[GCODE_NUM_AXES];  // Absolute position at end of segment.
                                       // In steps. (steps)

  // Derived values
  int delta_steps[GCODE_NUM_AXES];     // Difference to previous position. (steps)
  enum GCodeParserAxis defining_axis;  // index into defining axis.
  double speed;                        // Maximum speed of the defining axis (steps/s).
  double accel;                        // Maximum acceleration of the defining axis (steps/s^2).
  unsigned short aux_bits;             // Auxillary bits in this segment; set with M42
  double dx, dy, dz;                   // 3D delta_steps in real units (mm)
  double len;                          // 3D length (mm)
};
}  // end anonymous namespace

// Given the desired target speed(or acceleration) of the defining axis and the
// steps to be performed on all axes, determine if we need to scale down as to
// not exceed the individual maximum speed or acceleration constraints on any
// axis. Return the new defining axis limit based on the euclidean motion
// requested.
//
// Example with speed: given i the axis with the highest (relative) out of
// bounds speed, what's the speed of the defining_axis so that every speed
// respects i's bounds? We just need to project each speed to the defining
// axis speed, and pick the smallest.
//
// We need to work in mm, since steps is not a uniform unit of space across
// the different axes.
static float clamp_defining_axis_limit(const int *axes_steps,
                                       const FloatAxisConfig &axes_limits_mm,
                                       enum GCodeParserAxis defining_axis,
                                       const FloatAxisConfig &steps_per_mm) {
  float new_defining_axis_limit = axes_limits_mm[defining_axis];
  for (const GCodeParserAxis i : AllAxes()) {
    if (axes_steps[i] == 0) continue;
    const float ratio =
      std::fabs((1.0 * axes_steps[i] * steps_per_mm[defining_axis]) /
                (axes_steps[defining_axis] * steps_per_mm[i]));
    new_defining_axis_limit =
      std::min(new_defining_axis_limit, axes_limits_mm[i] / ratio);
  }
  return new_defining_axis_limit * steps_per_mm[defining_axis];
}

// Uniformly accelerated ramp distance estimation.
// The ramp is parametrized by v0 as the starting speed, v1 the final speed
// and the constant acceleration.
static double uar_distance(double v0, double v1, double acceleration) {
  return (v1 * v1 - v0 * v0) / (2 * acceleration);
}

// Uniformly accelerated ramp final speed estimation where
// s is the travelled distance, v0 the starting speed and
// acceleration is the constant acceleration.
static double uar_speed(double s, double v0, double acceleration) {
  return std::sqrt(2 * acceleration * s + v0 * v0);
}

// Compute the distance for an acceleration ramp to cross a deceleration
// ramp with the same module of the acceleration.
static float find_uar_intersection(double v0, double v1, double acceleration, double distance) {
  const double min_v = v0 <= v1 ? v0 : v1;
  const double max_v = v0 <= v1 ? v1 : v0;
  // Distance we would need to reach the maximum between the two speeds using the acceleration
  // provided.
  const double reaching_distance = uar_distance(min_v, max_v, acceleration);
  return reaching_distance * (v0 < v1) + (distance - reaching_distance) / 2;
}

class Planner::Impl {
 public:
  Impl(const MachineControlConfig *config, HardwareMapping *hardware_mapping,
       SegmentQueue *motor_backend);
  ~Impl();

  bool move_machine_steps(const struct AxisTarget *last_pos,
                          struct AxisTarget *target_pos,
                          const struct AxisTarget *upcoming);

  void assign_steps_to_motors(struct LinearSegmentSteps *command,
                              enum GCodeParserAxis axis, int steps);

  bool issue_motor_move_if_possible(bool flush_planning_queue = false);
  bool machine_move(const AxesRegister &axis, float feedrate);
  void bring_path_to_halt();

  // Avoid division by zero if there is no config defined for axis.
  double axis_delta_to_mm(const AxisTarget *pos, enum GCodeParserAxis axis) {
    if (cfg_->steps_per_mm[axis] != 0.0)
      return (double)pos->delta_steps[axis] / cfg_->steps_per_mm[axis];
    return 0.0;
  }

  double euclidian_speed(const struct AxisTarget *t);

  void GetCurrentPosition(AxesRegister *pos);
  int DirectDrive(GCodeParserAxis axis, float distance, float v0, float v1);
  void SetExternalPosition(GCodeParserAxis axis, float pos);

 private:
  const struct MachineControlConfig *const cfg_;
  HardwareMapping *const hardware_mapping_;
  SegmentQueue *const motor_ops_;

  struct PlanningSegment {
    AxisTarget target;

    // Currently planned profile for an AxisTarget.
    //    |  v1
    //    |   /¯¯¯¯¯¯¯¯¯¯\
    //    |  /            \ v2
    //    | /
    // v0 |/________________
    //      acc  travel   dec
    struct PlannedProfile {
      uint32_t accel, travel, decel; // Steps of the defining axis for each ramp
      float v0, v1, v2; // Speed of the defining axis,
                            // v0-v1 accel, v1-v2 travel, v2-v3 decel.
    } planned; // Planned segments
  };

  // Run backward and forward passes to
  // compute the new profile.
  void UpdateMotionProfile();

  // Next buffered positions. Written by incoming gcode, read by outgoing
  // motor movements.
  RingDeque<PlanningSegment, 256> planning_buffer_;

  // Initial speed point of the planning buffer in (steps/s)
  // for the defining axis of the first planning buffer segment.
  double start_speed_; // Starting speed of the defining axis of the first Planning
                       // Segment.
  int start_position_steps_[GCODE_NUM_AXES];  // Absolute starting position (steps)

  // Pre-calculated per axis limits in steps, steps/s, steps/s^2
  // All arrays are indexed by axis.
  AxesRegister max_axis_speed_;  // max travel speed hz
  AxesRegister max_axis_accel_;  // acceleration hz/s

  HardwareMapping::AuxBitmap last_aux_bits_;  // last enqueued aux bits.

  bool path_halted_;
  bool position_known_;
};

// Every AxisTarget, the defining axis might change. Speeds though, should stay continuous
// between each target for every axis. We need a function to compute what was the speed in the previous
// target for the new defining axis.
static double get_speed_factor_for_axis(const struct AxisTarget *t,
                                        enum GCodeParserAxis request_axis) {
  if (t->delta_steps[t->defining_axis] == 0) return 0.0;
  return (request_axis == t->defining_axis) ? 1.0 : 1.0 * t->delta_steps[request_axis] / t->delta_steps[t->defining_axis];
}

// Get the speed for a particular axis. Depending on the direction, this can
// be positive or negative.
static double get_speed_for_axis(const struct AxisTarget *target,
                                 enum GCodeParserAxis request_axis) {
  return target->speed * get_speed_factor_for_axis(target, request_axis);
}

// Given that we want to travel "s" steps, start with speed "v0",
// accelerate peak speed v1 and slow down to "v2" with acceleration "a",
// what is v1 ?
static double get_peak_speed(int s, double v0, double v2, double a) {
  return std::sqrt(v2 * v2 + v0 * v0 + 2 * a * s) / std::sqrt(2.0);
}

static double euclid_distance(double x, double y, double z) {
  return std::sqrt(x * x + y * y + z * z);
}

// Number of steps to accelerate or decelerate (negative "a") from speed
// v0 to speed v1. Modifies v1 if we can't reach the speed with the allocated
// number of steps.
static double steps_for_speed_change(double a, double v0, double *v1,
                                     int max_steps) {
  // s = v0 * t + a/2 * t^2
  // v1 = v0 + a*t
  const double t = (*v1 - v0) / a;
  // TODO:
  if (t < 0) Log_error("Error condition: t=%.1f INSUFFICIENT LOOKAHEAD\n", t);
  double steps = a / 2 * t * t + v0 * t;
  if (steps <= max_steps) return steps;
  // Ok, we would need more steps than we have available. We correct the speed
  // to what we actually can reach.
  *v1 = std::sqrt(v0 * v0 + 2 * a * max_steps);
  return max_steps;
}

// Returns true, if all results in zero movement
static bool subtract_steps(struct LinearSegmentSteps *value,
                           const struct LinearSegmentSteps &subtract) {
  bool has_nonzero = false;
  for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
    value->steps[i] -= subtract.steps[i];
    has_nonzero |= (value->steps[i] != 0);
  }
  return has_nonzero;
}

static bool within_acceptable_range(double new_val, double old_val,
                                    double fraction) {
  const double max_diff = fraction * old_val;
  if (new_val < old_val - max_diff) return false;
  if (new_val > old_val + max_diff) return false;
  return true;
}

// Determine the fraction of the speed that "from" should decelerate
// to at the end of its travel.
// The way trapezoidal moves work, be still have to decelerate to zero in
// most times, which is inconvenient. TODO(hzeller): speed matching is not
// cutting it :)
static double determine_joining_speed(const struct AxisTarget *from,
                                      const struct AxisTarget *to,
                                      const double threshold,
                                      const double speed_tune_angle) {
  // the dot product of the vectors
  const double dot = from->dx * to->dx + from->dy * to->dy + from->dz * to->dz;
  const double mag = from->len * to->len;
  if (dot == 0) return 0.0;  // orthogonal 90 degree, full stop
  if (dot < 0) return 0.0;   // turning around, full stop
  const double dotmag = dot / mag;
  if (within_acceptable_range(1.0, dotmag, 1e-5))
    return to->speed;  // codirectional 0 degree, keep accelerating

  // the angle between the vectors
  const double rad2deg = 180.0 / M_PI;
  const double angle = std::fabs(std::acos(dotmag) * rad2deg);

  if (angle >= 45.0) return 0.0;  // angle to large, come to full stop
  if (angle <= threshold) {       // in tolerance, keep accelerating
    if (dot < 1) {  // speed tune segments less than 1mm (i.e. arcs)
      const double deg2rad = M_PI / 180.0;
      const double angle_speed_adj =
        std::cos((angle + speed_tune_angle) * deg2rad);
      return to->speed * angle_speed_adj;
    }
    return to->speed;
  }

  // The angle between the from and to segments is < 45 degrees but greater
  // than the threshold. Use the "old" logic to determine the joining speed
  //
  // Our goal is to figure out what our from defining speed should
  // be at the end of the move.
  bool is_first = true;
  double from_defining_speed = from->speed;
  const int from_defining_steps = from->delta_steps[from->defining_axis];
  for (const GCodeParserAxis axis : AllAxes()) {
    const int from_delta = from->delta_steps[axis];
    const int to_delta = to->delta_steps[axis];

    // Quick integer decisions
    if (from_delta == 0 && to_delta == 0) continue;  // uninteresting: no move.
    if (from_delta == 0 || to_delta == 0) return 0.0;  // accel from/to zero
    if ((from_delta < 0 && to_delta > 0) || (from_delta > 0 && to_delta < 0))
      return 0.0;  // turing around

    double to_speed = get_speed_for_axis(to, axis);
    // What would this speed translated to our defining axis be ?
    double speed_conversion = 1.0 * from_defining_steps / from_delta;
    double goal = to_speed * speed_conversion;
    if (goal < 0.0) return 0.0;
    if (is_first || within_acceptable_range(goal, from_defining_speed, 1e-5)) {
      if (goal < from_defining_speed) from_defining_speed = goal;
      is_first = false;
    } else {
      return 0.0;  // Too far off.
    }
  }

  return from_defining_speed;
}

Planner::Impl::Impl(const MachineControlConfig *config,
                    HardwareMapping *hardware_mapping,
                    SegmentQueue *motor_backend)
    : cfg_(config),
      hardware_mapping_(hardware_mapping),
      motor_ops_(motor_backend),
      start_speed_(0),
      start_position_steps_{0},
      path_halted_(true),
      position_known_(true) {
  for (const GCodeParserAxis axis : AllAxes()) {
    HardwareMapping::AxisTrigger trigger = cfg_->homing_trigger[axis];
    const float home_pos =
      trigger == HardwareMapping::TRIGGER_MAX ? cfg_->move_range_mm[axis] : 0;
    SetExternalPosition(axis, home_pos);
  }
  position_known_ = true;

  for (const GCodeParserAxis i : AllAxes()) {
    max_axis_speed_[i] = cfg_->max_feedrate[i] * cfg_->steps_per_mm[i];
    const float accel = cfg_->acceleration[i] * cfg_->steps_per_mm[i];
    max_axis_accel_[i] = accel;
  }
}

Planner::Impl::~Impl() { bring_path_to_halt(); }

// Assign steps to all the motors responsible for given axis.
void Planner::Impl::assign_steps_to_motors(struct LinearSegmentSteps *command,
                                           enum GCodeParserAxis axis,
                                           int steps) {
  hardware_mapping_->AssignMotorSteps(axis, steps, command);
}

double Planner::Impl::euclidian_speed(const struct AxisTarget *t) {
  double speed_factor = 1.0;
  if (t->len > 0) {
    const double axis_len_mm = axis_delta_to_mm(t, t->defining_axis);
    speed_factor = std::fabs(axis_len_mm) / t->len;
  }
  return t->speed * speed_factor;
}

// Move the given number of machine steps for each axis.
//
// This will be up to three segments: accelerating from last_pos speed to
// target speed, regular travel, and decelerating to the speed that the
// next segment is never forced to decelerate, but stays at speed or accelerate.
//
// The segments are sent to the motor operations backend.
//
// Since we calculate the deceleration, this modifies the speed of target_pos
// to reflect what the last speed was at the end of the move.
//
// Returns true if move was executed, false if aborted
bool Planner::Impl::move_machine_steps(const struct AxisTarget *last_pos,
                                       struct AxisTarget *target_pos,
                                       const struct AxisTarget *upcoming) {
  bool ret = true;
  if (target_pos->delta_steps[target_pos->defining_axis] == 0) {
    if (last_aux_bits_ != target_pos->aux_bits) {
      // Special treatment: bits changed since last time, let's push them
      // through.
      struct LinearSegmentSteps bit_set_command = {};
      bit_set_command.aux_bits = target_pos->aux_bits;
      ret = motor_ops_->Enqueue(bit_set_command);
      last_aux_bits_ = target_pos->aux_bits;
    }
    return ret;
  }
  struct LinearSegmentSteps accel_command = {};
  struct LinearSegmentSteps move_command = {};
  struct LinearSegmentSteps decel_command = {};

  assert(target_pos->speed > 0);  // Speed is always a positive scalar.

  // Aux bits are set synchronously with what we need.
  move_command.aux_bits = target_pos->aux_bits;
  const enum GCodeParserAxis defining_axis = target_pos->defining_axis;

  // Common settings.
  memcpy(&accel_command, &move_command, sizeof(accel_command));
  memcpy(&decel_command, &move_command, sizeof(decel_command));

  // Always start from the last steps/sec speed to avoid motion glitches.
  // The planner will use that speed to determine what the peak speed for
  // this move is and if we need to accel to reach the desired target speed.
  const double last_speed = last_pos->speed;

  // We need to arrive at a speed that the upcoming move does not have
  // to decelerate further (after all, it has a fixed feed-rate it should not
  // go over).
  double next_speed = determine_joining_speed(
    target_pos, upcoming, cfg_->threshold_angle, cfg_->speed_tune_angle);
  // Clamp the next speed to insure that this segment does not go over.
  if (next_speed > target_pos->speed) next_speed = target_pos->speed;

  const int *axis_steps = target_pos->delta_steps;  // shortcut.
  const int abs_defining_axis_steps = abs(axis_steps[defining_axis]);

  // Max acceleration in (steps/s^2)
  const double a = clamp_defining_axis_limit(axis_steps, cfg_->acceleration,
                                             defining_axis, cfg_->steps_per_mm);
  const double peak_speed =
    get_peak_speed(abs_defining_axis_steps, last_speed, next_speed, a);
  assert(peak_speed > 0);

  // TODO: if we only have < 5 steps or so, we should not even consider
  // accelerating or decelerating, but just do one speed.

  if (peak_speed < target_pos->speed) {
    target_pos->speed = peak_speed;  // Don't manage to accelerate to desired v
  }

  // Make sure the target feedrate for the move is clamped to what all the
  // moving axes can reach.
  const double max_speed = clamp_defining_axis_limit(
    axis_steps, cfg_->max_feedrate, defining_axis, cfg_->steps_per_mm);
  if (max_speed < target_pos->speed) target_pos->speed = max_speed;

  const double accel_fraction =
    (last_speed < target_pos->speed)
      ? steps_for_speed_change(a, last_speed, &target_pos->speed,
                               abs_defining_axis_steps) /
          abs_defining_axis_steps
      : 0;

  // We only decelerate if the upcoming speed is _slower_
  double dummy_next_speed = next_speed;  // Don't care to modify; we don't have
  const double decel_fraction =
    (next_speed < target_pos->speed)
      ? steps_for_speed_change(-a, target_pos->speed, &dummy_next_speed,
                               abs_defining_axis_steps) /
          abs_defining_axis_steps
      : 0;

#if 0
  // Useful debugging info.
  // TODO: we get a decel glitch when the last_pos has accelerated to a speed
  // that is faster than the target can decelerate from.
  if (dummy_next_speed != next_speed) {
    fprintf(stderr, "  \033[1m\033[31mGLITCH\033[0m "
                    "- move too short for full decel (reached v1: %10.2f)\n",
            dummy_next_speed);
  }
#endif

  assert(accel_fraction + decel_fraction <= 1.0 + 1e-3);

#if 0
  // fudging: if we have tiny acceleration segments, don't do these at all
  // but only do speed; otherwise we have a lot of rattling due to many little
  // segments of acceleration/deceleration (e.g. for G2/G3).
  // This is not optimal. Ideally, we would actually calculate in terms of
  // jerk and optimize to stay within that constraint.
  const int accel_decel_steps
    = (accel_fraction + decel_fraction) * abs_defining_axis_steps;
  const double accel_decel_mm
    = (accel_decel_steps / cfg_->steps_per_mm[defining_axis]);
  const char do_accel = (accel_decel_mm > 2 || accel_decel_steps > 16);
#else
  const char do_accel = 1;
#endif

  bool has_accel = false;
  bool has_move = false;
  bool has_decel = false;

  if (do_accel && accel_fraction * abs_defining_axis_steps > 0) {
    has_accel = true;
    accel_command.v0 = (float)last_speed;         // Last speed of defining axis
    accel_command.v1 = (float)target_pos->speed;  // New speed of defining axis

    // Now map axis steps to actual motor driver
    for (const GCodeParserAxis a : AllAxes()) {
      const int accel_steps = std::lround(accel_fraction * axis_steps[a]);
      assign_steps_to_motors(&accel_command, a, accel_steps);
    }
  } else {
    if (last_speed)
      target_pos->speed = last_speed;  // No accel so use the last speed
  }

  move_command.v0 = (float)target_pos->speed;
  move_command.v1 = (float)target_pos->speed;

  if (do_accel && decel_fraction * abs_defining_axis_steps > 0) {
    has_decel = true;
    decel_command.v0 = (float)target_pos->speed;
    decel_command.v1 = (float)next_speed;
    target_pos->speed = next_speed;

    // Now map axis steps to actual motor driver
    for (const GCodeParserAxis a : AllAxes()) {
      const int decel_steps = std::lround(decel_fraction * axis_steps[a]);
      assign_steps_to_motors(&decel_command, a, decel_steps);
    }
  }

  // Move is everything that hasn't been covered in speed changes.
  // So we start with all steps and subtract steps done in acceleration and
  // deceleration.
  for (const GCodeParserAxis a : AllAxes()) {
    assign_steps_to_motors(&move_command, a, axis_steps[a]);
  }
  subtract_steps(&move_command, accel_command);
  has_move = subtract_steps(&move_command, decel_command);

  if (cfg_->synchronous) motor_ops_->WaitQueueEmpty();

  // Make sure each segment gets added in case we get aborted
  if (has_accel) ret = motor_ops_->Enqueue(accel_command);
  if (ret && has_move) ret = motor_ops_->Enqueue(move_command);
  if (ret && has_decel) ret = motor_ops_->Enqueue(decel_command);

  last_aux_bits_ = target_pos->aux_bits;

  return ret;
}

bool Planner::Impl::issue_motor_move_if_possible() {
  bool ret = true;
  if (planning_buffer_.size() >= 3) {
    ret =
      move_machine_steps(planning_buffer_[0],   // Current established position.
                         planning_buffer_[1],   // Position we want to move to.
                         planning_buffer_[2]);  // Next upcoming.
    planning_buffer_.pop_front();
  }
  return ret;
}

bool Planner::Impl::machine_move(const AxesRegister &axis, float feedrate) {
  assert(position_known_);  // call SetExternalPosition() after DirectDrive()

  // We always have a previous position.
  int *previous_position_steps = start_position_steps_;
  if (planning_buffer_.size() > 0) {
    previous_position_steps = planning_buffer_.back()->target.position_steps;
  }

  struct AxisTarget *new_pos = &planning_buffer_.append()->target;
  int max_steps = -1;
  enum GCodeParserAxis defining_axis = AXIS_X;

  // Real world -> machine coordinates. Here, we are rounding to the next full
  // step, but we never accumulate the error, as we always use the absolute
  // position as reference.
  for (const GCodeParserAxis a : AllAxes()) {
    new_pos->position_steps[a] = std::lround(axis[a] * cfg_->steps_per_mm[a]);
    new_pos->delta_steps[a] =
      new_pos->position_steps[a] - previous_position_steps[a];

    // The defining axis is the one that has to travel the most steps. It
    // defines the frequency to go. All the other axes are doing a fraction of
    // the defining axis.
    if (abs(new_pos->delta_steps[a]) > max_steps) {
      max_steps = abs(new_pos->delta_steps[a]);
      defining_axis = a;
    }
  }

  if (max_steps == 0) {
    // Nothing to do, ignore this move.
    planning_buffer_.pop_back();
    return true;
  }

  assert(max_steps > 0);

  new_pos->aux_bits = hardware_mapping_->GetAuxBits();
  new_pos->defining_axis = defining_axis;

  // Work out the real units values for the euclidian axes now to avoid
  // having to replicate the calcs later.
  new_pos->dx = axis_delta_to_mm(new_pos, AXIS_X);
  new_pos->dy = axis_delta_to_mm(new_pos, AXIS_Y);
  new_pos->dz = axis_delta_to_mm(new_pos, AXIS_Z);
  new_pos->len = euclid_distance(new_pos->dx, new_pos->dy, new_pos->dz);

  // Convert the arc-length feedrate (hypotenuse) from mm/s
  // to the defininx axis step/s.
  new_pos->speed = feedrate * cfg_->steps_per_mm[defining_axis];

  // Project the euclidean speed to the defining axis leg.
  new_pos->speed = euclidian_speed(new_pos);

  // Make sure the target feedrate for the move is clamped to what all the
  // moving axes can reach.
  const double max_speed = clamp_defining_axis_limit(
    new_pos->delta_steps, cfg_->max_feedrate, defining_axis, cfg_->steps_per_mm);
  if (max_speed < new_pos->speed) new_pos->speed = max_speed;

  // Define the maximum acceleration given the following motion angles and
  // defining axis.
  new_pos->accel = clamp_defining_axis_limit(new_pos->delta_steps, cfg_->acceleration,
                                             defining_axis, cfg_->steps_per_mm);

  // Run the planning algorithm.
  // Update the planning_buffer_.planned struct, as well as
  // redefine starting and final speeds.
  UpdateMotionProfile();

  // Send the slots until we reach the final deceleration ramp.
  // The ramp can cover more than one segment.
  bool ret = issue_motor_move_if_possible();
  if (ret) path_halted_ = false;
  return ret;
}

void Planner::Impl::bring_path_to_halt() {
  if (path_halted_) return;

  // Flush the queue.
  issue_motor_move_if_possible();
  path_halted_ = true;
}

// Backward and forward passes.
void Planner::Impl::UpdateMotionProfile() {
  // Starting speed for each Planning segment.
  double previous_speed = start_speed_;
  GCodeParserAxis defining_axis;
  PlanningSegment *segment;

  // Amount of steps necessary to reach a target speed.
  uint32_t roof_steps;

  // Forward pass. Compute only acceleration ramps.
  for (int i = 0; i < planning_buffer_.size(); ++i) {
    segment = planning_buffer_[i];

    // Get the current defining axis
    defining_axis = segment->target.defining_axis;

    // Previous speed is the speed of the defining axis previous segment.
    // We want to have continuous speeds. We calculate what would be that speed
    // for the new defininx axis.
    previous_speed /= get_speed_factor_for_axis(&segment->target, defining_axis);

    const double speed = segment->target.speed;
    const double max_accel = segment->target.accel;
    const int steps = segment->target.delta_steps[defining_axis];

    if (speed <= previous_speed) {
      previous_speed = speed;
      continue;
    }

    // Compute how many steps would be required to reach the target feedrate
    // (roof) from the previous_speed with the maximum acceleration.
    // We intentionally round toward zero (floor, steps always >0).
    roof_steps = (uint32_t) uar_distance(previous_speed, speed,
                                         max_accel);
    roof_steps = (roof_steps <= steps) ? roof_steps : steps;
    segment->planned.accel = roof_steps;
    segment->planned.v0 = previous_speed;
    const double v1 = uar_speed(roof_steps, previous_speed, max_accel);
    segment->planned.v1 = v1;
    previous_speed = v1;
  }

  previous_speed = 0;

  // Perform the backward pass. Compute and join the deceleration ramps
  // (which, by running backward become acceleration ramps), and travel segments
  // to the acceleration ramps or target feedrates.
  for (int i = planning_buffer_.size() - 1; i >= 0; --i) {
    // Let's see how many steps are required to reach the target feedrate.
    // Since we are computing a deceleration ramp, the previous feedrate is v0
    // while the target feedrate is v.
    segment = planning_buffer_[i];

    // Previous speed is the speed of the defining axis previous segment.
    // We want to have continuous speeds. We calculate what would be that speed
    // for the new defininx axis.
    previous_speed =
      previous_speed / get_speed_factor_for_axis(&segment->target,
                                                 defining_axis);

    defining_axis = segment->target.defining_axis;
    const double speed = segment->target.speed;
    const double max_accel = segment->target.accel;
    const int steps = segment->target.delta_steps[defining_axis];

    roof_steps = (int) uar_distance(previous_speed, speed, max_accel);

    // If there's no acceleration ramp steps should be allocated either
    // as deceleration ramp first, or as travel.
    if (segment->planned.accel == 0) {
      const uint32_t decel_steps =  (roof_steps >= steps) ? steps : roof_steps;
      segment->planned.decel = decel_steps;
      segment->planned.v2 = previous_speed;
      const double v0 = uar_speed(decel_steps, previous_speed, max_accel);
      segment->planned.v1 = v0;

      // Let's do some travel
      segment->planned.travel = steps - decel_steps;
      segment->planned.v1 = v0;

      // Set the new previous feedrate.
      previous_speed = v0;
      continue;
    }

    // In this case, there's an acceleration ramp, and it matches
    // already with the previous speed.
    if (previous_speed == segment->planned.v1) {
      segment->planned.travel = steps - segment->planned.accel;
      segment->planned.v1 = previous_speed;
      previous_speed = segment->planned.v0;
      continue;
    }

    uint32_t intersection = (int) find_uar_intersection(segment->planned.v0, previous_speed, max_accel, steps);

    // We stay below the acceleration ramp, we just have deceleration.
    if (intersection <= 0) {
      segment->planned.accel = 0;
      segment->planned.travel = 0;
      segment->planned.decel = steps;
      const double v1 = uar_speed(steps, previous_speed, max_accel);
      segment->planned.v1 = v1;
      segment->planned.v2 = previous_speed;
      previous_speed = v1;
      continue;
    }

    // Intersection of accel/decel ramps.
    if (intersection <= segment->planned.accel) {
      segment->planned.accel = intersection;
      const double v1 = uar_speed(segment->planned.accel, segment->planned.v0, max_accel);
      segment->planned.v1 = v1;
      segment->planned.travel = 0;
      segment->planned.decel = steps - intersection;
      segment->planned.v1 = v1;
      segment->planned.v2 = previous_speed;
      previous_speed = segment->planned.v0;
      continue;
    }

    // Accel-travel decel
    segment->planned.travel = steps - segment->planned.accel - roof_steps;
    segment->planned.v2 = previous_speed;
    segment->planned.decel = steps - intersection;
    previous_speed = segment->planned.v0;
  }
}

void Planner::Impl::GetCurrentPosition(AxesRegister *pos) {
  pos->zero();
  PhysicalStatus physical_status;
  if (!motor_ops_->GetPhysicalStatus(&physical_status))
    return;  // Should we return boolean to indicate that not supported ?
  for (const GCodeParserAxis a : AllAxes()) {
#if M114_DEBUG
    Log_debug("Machine steps Axis %c : %8d\n", gcodep_axis2letter(a),
              hardware_mapping_->GetAxisSteps(a, physical_status));
#endif
    if (cfg_->steps_per_mm[a] != 0) {
      (*pos)[a] = hardware_mapping_->GetAxisSteps(a, physical_status) /
                  cfg_->steps_per_mm[a];
    }
  }
}

int Planner::Impl::DirectDrive(GCodeParserAxis axis, float distance, float v0,
                               float v1) {
  bring_path_to_halt();  // Precondition. Let's just do it for good measure.
  position_known_ = false;

  const float steps_per_mm = cfg_->steps_per_mm[axis];

  struct LinearSegmentSteps move_command = {};

  move_command.v0 = v0 * steps_per_mm;
  if (move_command.v0 > max_axis_speed_[axis])
    move_command.v0 = max_axis_speed_[axis];
  move_command.v1 = v1 * steps_per_mm;
  if (move_command.v1 > max_axis_speed_[axis])
    move_command.v1 = max_axis_speed_[axis];

  move_command.aux_bits = hardware_mapping_->GetAuxBits();

  const int segment_move_steps = std::lround(distance * steps_per_mm);
  assign_steps_to_motors(&move_command, axis, segment_move_steps);

  motor_ops_->Enqueue(move_command);
  motor_ops_->WaitQueueEmpty();

  return segment_move_steps;
}

void Planner::Impl::SetExternalPosition(GCodeParserAxis axis, float pos) {
  assert(path_halted_);  // Precondition.
  position_known_ = true;

  const int motor_position = std::lround(pos * cfg_->steps_per_mm[axis]);
  planning_buffer_.back()->target.position_steps[axis] = motor_position;
  planning_buffer_[0]->target.position_steps[axis] = motor_position;

  const uint8_t motormap_for_axis = hardware_mapping_->GetMotorMap(axis);
  for (int motor = 0; motor < BEAGLEG_NUM_MOTORS; ++motor) {
    if (motormap_for_axis & (1 << motor))
      motor_ops_->SetExternalPosition(motor, motor_position);
  }
}

// -- public interface

Planner::Planner(const MachineControlConfig *config,
                 HardwareMapping *hardware_mapping, SegmentQueue *motor_backend)
    : impl_(new Impl(config, hardware_mapping, motor_backend)) {}

Planner::~Planner() { delete impl_; }

bool Planner::Enqueue(const AxesRegister &target_pos, float speed) {
  return impl_->machine_move(target_pos, speed);
}

void Planner::BringPathToHalt() { impl_->bring_path_to_halt(); }

void Planner::GetCurrentPosition(AxesRegister *pos) {
  impl_->GetCurrentPosition(pos);
}

int Planner::DirectDrive(GCodeParserAxis axis, float distance, float v0,
                         float v1) {
  return impl_->DirectDrive(axis, distance, v0, v1);
}

void Planner::SetExternalPosition(GCodeParserAxis axis, float pos) {
  impl_->SetExternalPosition(axis, pos);
}
