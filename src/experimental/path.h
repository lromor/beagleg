#ifndef __PATH_H_
#define __PATH_H_
#include <cmath>
#include <string>
#include <sstream>
#include <memory>
#include <array>
#include <ostream>
#include "vector.h"
#include <math.h>
#include "bezier.h"

using FloatVector3D = Vector<3, float>;
using IntVector3D = Vector<3, int>;
using PositionSteps = IntVector3D;

// Basic trajectory arc types.
enum class PathPrimitiveType { Line, CubicBezier, CircularArc };

// Define an abstract piece of trajectory.
// The parameter template N is the dimensionality of the trajectory.
template<size_t N>
struct PathPrimitive {
  using Type = PathPrimitiveType;
  PositionSteps end;
  double speed;
  virtual const double ComputeArcLength(const PositionSteps &start) const = 0;
  virtual const Type GetType() const = 0;
};

template<size_t N>
struct ArcPathPrimitive : public PathPrimitive<N> {
  constexpr static PathPrimitiveType type = PathPrimitiveType::CircularArc;
  // Center position relative to the start.
  FloatVector3D center;
  virtual double ComputeArcLength(const PositionSteps &start) const final {
    const double chord_length = (this->end - start).norm();
    const double ray_length = center.norm();
    return ray_length * 2 * std::asin(ray_length / chord_length);
  }
  virtual const PathPrimitiveType GetType() const final { return type; }
};

template<size_t N>
struct LinePathPrimitive : public PathPrimitive<N> {
  constexpr static PathPrimitiveType type = PathPrimitiveType::Line;
  virtual const double ComputeArcLength(const PositionSteps &start) const final {
    return (this->end - start).norm();
  }
  virtual const PathPrimitiveType GetType() const final { return type; }
};

template<size_t N>
struct CubicBezierPathPrimitive : public PathPrimitive<N> {
  constexpr static PathPrimitiveType type = PathPrimitiveType::CubicBezier;
  // cp0 and cp3 are the previous point, and the end.
  FloatVector3D cp1, cp2;
  virtual const double ComputeArcLength(const PositionSteps &start) const final {
    const auto b = CubicBezier<3, float>{start, cp1, cp2, this->end};
    return b.length();
  }
  virtual const PathPrimitiveType GetType() const final { return type; }
};

typedef std::unique_ptr<PathPrimitive<3>> UniquePathPrimitive;
#endif // __PATH_H_
