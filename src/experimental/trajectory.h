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

template<size_t N>
struct TrajectoryPrimitive {
  double requested_feedrate;
  Vector<N> end;
  virtual double length() const = 0;
};

enum class TrajectoryPrimitiveType { Line, CubicBezier, Arc };

template<size_t N>
struct ArcTrajectoryPrimitive : public TrajectoryPrimitive<N> {
  Vector<N> center;

  double length() const final {
    double chord_length = norm(this->end);
    double ray_length = norm(center);
    return ray_length * 2 * std::asin(ray_length / chord_length);
  }

  constexpr static TrajectoryPrimitiveType type = TrajectoryPrimitiveType::Arc;
  std::string ToString() const {
    std::ostringstream ss;
    ss << "{ ";
    ss << "type: arc" << ", ";
    ss << "requested_feedrate: " << this->requested_feedrate << ", ";
    ss << "end: " << this->end << ", ";
    ss << "center: " << center;
    ss << " }";
    return ss.str();
  }
};

template<size_t N>
struct LineTrajectoryPrimitive : public TrajectoryPrimitive<N> {
  // Returns the arc-length of the primitive.
  double length() const {
    double d = 0;
    for (size_t i = 0; i < N; ++i) {
      auto v = this->end[i];
      d += v * v;
    }
    return std::sqrt(d);
  }

  constexpr static TrajectoryPrimitiveType type = TrajectoryPrimitiveType::Line;
  std::string ToString() const {
    std::ostringstream ss;
    ss << "{ ";
    ss << "type: line" << ", ";
    ss << "requested_feedrate: " << this->requested_feedrate << ", ";
    ss << "end: " << this->end;
    ss << " }";
    return ss.str();
  }
};

template<size_t N>
struct CubicBezierTrajectoryPrimitive : public TrajectoryPrimitive<N> {
  constexpr static TrajectoryPrimitiveType type = TrajectoryPrimitiveType::CubicBezier;
  Vector<N> cp1;
  Vector<N> cp2;
  Vector<N> cp3;

  // Quite an approximation.
  double length() const final {
    double chord = norm(cp3);
    double cont_net = norm(cp1) + norm(cp2 - cp1) + norm(cp3 - cp2);
    return (cont_net + chord) / 2;
  }

  std::string ToString() const {
    std::ostringstream ss;
    ss << "{ ";
    ss << "type: cubic_bezier" << ", ";
    ss << "requested_feedrate: " << this->requested_feedrate << ", ";
    ss << "end: [" << this->end << ", ";
    ss << "cp0" << 0 << ", ";
    ss << "cp1" << cp1 << ", ";
    ss << "cp2" << cp2 << ", ";
    ss << "cp3" << cp3;
    ss << " }";
    return ss.str();
  }
};

struct TrajectorySegment {
  std::unique_ptr<TrajectoryPrimitive<3>> curve;
  TrajectoryPrimitiveType type;
};

#endif // __TRAJECTORY_H_
