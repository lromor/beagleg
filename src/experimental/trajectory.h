#ifndef __TRAJECTORY_H_
#define __TRAJECTORY_H_
#include <string>
#include <sstream>
#include <memory>
#include <array>
#include <ostream>

template<size_t N>
using Point = std::array<double, N>;

template<size_t N>
struct TrajectoryPrimitive {
  double requested_feedrate;
  Point<N> end;
};

template<size_t N>
std::ostream& operator<< (std::ostream& os, const Point<N> &s) {
  os << "[";
  for (int i = 0; i < s.size(); ++i) {
    os << s[i];
    if (i == s.size() - 1) break;
    os << ", ";
  }
  os << "]";
  return os;
}

enum class TrajectoryPrimitiveType { Line, CubicBezier, Arc };

template<size_t N>
struct ArcTrajectoryPrimitive : public TrajectoryPrimitive<N> {
  Point<N> center;
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

template<unsigned O, size_t N>
struct Bezier {
  std::array<Point<N>, O + 1> cps;
};

template<size_t N>
using CubicBezier = Bezier<3, N>;

template<size_t N>
struct CubicBezierTrajectoryPrimitive : public TrajectoryPrimitive<N> {
  constexpr static TrajectoryPrimitiveType type = TrajectoryPrimitiveType::CubicBezier;
  Point<N> cp1;
  Point<N> cp2;
  Point<N> cp3;

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
