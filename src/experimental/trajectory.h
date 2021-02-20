#ifndef __TRAJECTORY_H_
#define __TRAJECTORY_H_
#include <cmath>
#include <string>
#include <sstream>
#include <memory>
#include <array>
#include <ostream>
#include "vector.h"
#include <math.h>
#include "bezier.h"

enum class TrajectoryPrimitiveType { Line, CubicBezier, CircularArc };

template<size_t N>
struct TrajectoryPrimitive {
  using Type = TrajectoryPrimitiveType;
  Vector<N> end;
  virtual double length() const = 0;
  virtual ~TrajectoryPrimitive() {}
};


template<size_t N>
struct ArcTrajectoryPrimitive : public TrajectoryPrimitive<N> {
  constexpr static TrajectoryPrimitiveType type = TrajectoryPrimitiveType::CircularArc;

  Vector<N> center;

  double length() const final {
    double chord_length = this->end.norm();
    double ray_length = center.norm();
    return ray_length * 2 * std::asin(ray_length / chord_length);
  }

  std::string ToString() const {
    std::ostringstream ss;
    ss << "{ ";
    ss << "type: circular_arc" << ", ";
    ss << "end: " << this->end << ", ";
    ss << "center: " << center;
    ss << " }";
    return ss.str();
  }
};

template<size_t N>
struct LineTrajectoryPrimitive : public TrajectoryPrimitive<N> {
  constexpr static TrajectoryPrimitiveType type = TrajectoryPrimitiveType::Line;

  double length() const {
    double d = 0;
    for (size_t i = 0; i < N; ++i) {
      auto v = this->end[i];
      d += v * v;
    }
    return std::sqrt(d);
  }

  std::string ToString() const {
    std::ostringstream ss;
    ss << "{ ";
    ss << "type: line" << ", ";
    ss << "end: " << this->end;
    ss << " }";
    return ss.str();
  }
};

template<size_t N>
struct CubicBezierTrajectoryPrimitive : public TrajectoryPrimitive<N> {
  constexpr static TrajectoryPrimitiveType type = TrajectoryPrimitiveType::CubicBezier;

  Vector<N> cp1, cp2, cp3;

  double length() const final {
    const auto b = CubicBezier<3>{cp1, cp2, cp3};
    return b.length();
  }

  std::string ToString() const {
    std::ostringstream ss;
    ss << "{ ";
    ss << "type: cubic_bezier" << ", ";
    ss << "end: [" << this->end << ", ";
    ss << "cp0" << 0 << ", ";
    ss << "cp1" << cp1 << ", ";
    ss << "cp2" << cp2 << ", ";
    ss << "cp3" << cp3;
    ss << " }";
    return ss.str();
  }
};

struct TrajectoryArc {
  size_t id; // Identifies the gcode command.
  std::unique_ptr<TrajectoryPrimitive<3>> curve;
  double requested_feedrate;
};

#endif // __TRAJECTORY_H_
