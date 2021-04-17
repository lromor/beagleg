#include "experimental-planner.h"

int main(int argc, char *argv[]) {
  GCodeParser::Config parser_cfg;
  MachineControlConfig config;
  ConfigParser config_parser;

  config_parser.SetContentFromFile("../../sample.config");
  config.ConfigureFromFile(&config_parser);

  config.require_homing = false;
  config.range_check = false;

  ExperimentalPlanner *planner = ExperimentalPlanner::Create(&config);
  GCodeParser parser(parser_cfg, planner->ParseEventReceiver());
  parser.ReadFile(stdin, stderr);

  delete planner;
  return 0;
}
