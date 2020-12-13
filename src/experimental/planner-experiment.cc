#include <iostream>
#include <queue>
#include <cassert>

#include <memory>
#include <sstream>

#include "../gcode-parser/gcode-parser.h"
#include "../gcode-machine-control.h"
#include "../config-parser.h"

enum class CurvePrimitiveType { Line, CubicBezier, Arc };

struct Point {
  double x, y, z;
  std::string ToString() const {
    std::ostringstream ss;
    ss << "{ x: " << x << ", y: " << y << ", z: " << z << " }";
    return ss.str();
  }
};

struct CurvePrimitive {
  double requested_feedrate;
  Point end;
};

struct Arc : public CurvePrimitive {
  struct Point center;
  constexpr static CurvePrimitiveType type = CurvePrimitiveType::Arc;
  std::string ToString() const {
    std::ostringstream ss;
    ss << "{ ";
    ss << "type: arc" << ", ";
    ss << "requested_feedrate: " << requested_feedrate << ", ";
    ss << "end: " << end.ToString() << ", ";
    ss << "center: " << center.ToString();
    ss << " }";
    return ss.str();
  }
};

struct Line : public CurvePrimitive {
  constexpr static CurvePrimitiveType type = CurvePrimitiveType::Line;
  std::string ToString() const {
    std::ostringstream ss;
    ss << "{ ";
    ss << "type: line" << ", ";
    ss << "requested_feedrate: " << requested_feedrate << ", ";
    ss << "end: " << end.ToString();
    ss << " }";
    return ss.str();
  }
};

struct CubicBezier : public CurvePrimitive {
  constexpr static CurvePrimitiveType type = CurvePrimitiveType::CubicBezier;
  Point cp1, cp2;

  std::string ToString() const {
    std::ostringstream ss;
    ss << "{ ";
    ss << "type: cubic_bezier" << ", ";
    ss << "requested_feedrate: " << requested_feedrate << ", ";
    ss << "end: " << end.ToString() << ", ";
    ss << "cp1" << cp1.ToString() << ", ";
    ss << "cp2" << cp1.ToString();
    ss << " }";
    return ss.str();
  }
};

class Planner {
public:

  Planner(double tolerance_mm = 0.1f) : tolerance_mm_(tolerance_mm) {}

  struct QueueElement {
    std::unique_ptr<CurvePrimitive> curve;
    CurvePrimitiveType type;
  };

  template<typename CurvePrimitiveClass>
  bool Enqueue(const CurvePrimitiveClass &v) {
    elements.push({std::unique_ptr<CurvePrimitive>(new CurvePrimitiveClass(v)), CurvePrimitiveClass::type});
    std::cout << v.ToString() << std::endl;
    return true;
  }

private:
  std::queue<QueueElement> elements;
  double tolerance_mm_;
};


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
    struct Line line;
    line.end = Point{absolute_pos[AXIS_X], absolute_pos[AXIS_Y], absolute_pos[AXIS_Z]};
    line.requested_feedrate = feed_mm_p_sec;
    planner_->Enqueue(line);
    return true;
  }
  virtual bool rapid_move(float feed_mm_p_sec,
                          const AxesRegister &absolute_pos) {
    struct Line line;
    line.end = Point{absolute_pos[AXIS_X], absolute_pos[AXIS_Y], absolute_pos[AXIS_Z]};
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

int main(int argc, char *argv[]) {
  GCodeParser::Config parser_cfg;
  Planner planner;

  MachineControlConfig config;
  ConfigParser config_parser;


  config_parser.SetContentFromFile("../../sample.config");
  config.ConfigureFromFile(&config_parser);

  config.require_homing = false;
  config.range_check = false;

  GCodeEventReceiver event_receiver(&planner);
  GCodeParser parser(parser_cfg, &event_receiver);

  parser.ReadFile(stdin, stderr);

  return 0;
}
