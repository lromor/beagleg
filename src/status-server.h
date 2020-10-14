#ifndef STATUS_SERVER_
#define  STATUS_SERVER_

#include <memory>
#include "gcode-parser/gcode-parser.h"
#include "common/fd-mux.h"
#include "gcode-machine-control.h"

class StatusServer {
public:
  static StatusServer *Create(FDMultiplexer *event_server,
                              const char *bind_addr, int port);
  ~StatusServer();
  GCodeMachineControl::StatusEventReceiver *StatusEventReceiver();

private:
  class Impl;
  StatusServer(Impl *impl);
  Impl *const impl_;
};

#endif //  STATUS_SERVER_
