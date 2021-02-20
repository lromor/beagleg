#include "planner-experiment.h"

int main(int argc, char *argv[]) {
  GCodeParser::Config parser_cfg;
  MachineControlConfig config;
  Planner *planner = Planner::Create(&config);

  ConfigParser config_parser;


  config_parser.SetContentFromFile("../../sample.config");
  config.ConfigureFromFile(&config_parser);

  config.require_homing = false;
  config.range_check = false;

  GCodeEventReceiver event_receiver(planner);
  GCodeParser parser(parser_cfg, &event_receiver);

  parser.ReadFile(stdin, stderr);
  return 0;
}
