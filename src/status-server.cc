#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <unistd.h>
#include <microhttpd.h>
#include <set>
#include <sstream>
#include "common/fd-mux.h"
#include "gcode-parser/gcode-parser.h"
#include "status-server.h"
#include "gcode-machine-control.h"
#include "common/logging.h"

template<typename T>
static inline void reverse_memcpy(T * dst, const T *src, size_t size) {
  for (size_t i = 0; i < size; ++i)
    dst[size - 1 - i] = src[i];
}

template<typename T>
class OutputBuffer {
public:
  OutputBuffer() : data_(NULL), size_(0) {}

  ~OutputBuffer() { clear(); }

  bool Update(const T *data, const size_t size){
    if (data_ == NULL) {
      data_ = (T *)malloc(sizeof(T) * size);
      if (data_ == NULL) {
        perror("malloc()");
        return false;
      }
    } else {
      const size_t new_size = size_ + size;
      data_ = (T *)realloc(data_, new_size);
      if (data_ == NULL) {
        perror("realloc()");
        return false;
      }
      // Shift previous data
      memcpy(data_ + size, data_, size_);
      if (data_ == NULL) {
        perror("memcpy()");
        return false;
      }
    }

    reverse_memcpy(data_, data, size);
    size_ += size;
    return true;
  }

  const size_t Consume(void *data, const size_t size_bytes) {
    const size_t current_bytes = bytes();
    const size_t data_size_bytes =
      (size_bytes > current_bytes) ? current_bytes : size_bytes;
    const size_t start = (current_bytes - data_size_bytes) / sizeof(T);
    reverse_memcpy((T *)data, (T *)data_ + start, data_size_bytes / sizeof(T));
    if (start > 0) {
      data_ = (T *)realloc(data_, size_);
      if (data_ == NULL) {
        perror("realloc()");
        return 0;
      }
    }
    size_ = start;
    return data_size_bytes;
  }

  const size_t size() { return size_; }
  const size_t bytes() { return size() * sizeof(T); }
  void clear() { free(data_); size_ = 0; }

private:
  T *data_;
  size_t size_;
};

class MHDServer : public FDMultiplexer::ExternalHandler {
public:
  MHDServer(const int port) : d_(NULL) {
    d_ = MHD_start_daemon(
      MHD_USE_DEBUG, port, NULL, NULL,
      &AccessHandlerCallbackWrapper, (void *) this,
      MHD_OPTION_NOTIFY_CONNECTION, &NotifyConnectionCallbackWrapper, (void *) this,
      MHD_OPTION_END);
  }

  virtual bool Register(fd_set *read_fds, fd_set *write_fds,
                        fd_set *except_fds, int *max_fd) final {

    int max = 0;
    MHD_get_fdset(d_, read_fds, write_fds, except_fds, &max);
    *max_fd = std::max(max, *max_fd);
    return true;
  }

  virtual bool Trigger(fd_set *read_fds, fd_set *write_fds, fd_set *except_fds) final {
    MHD_run_from_select(d_, read_fds, write_fds, except_fds);
    return true;
  }

  bool SendData(const std::string &data) {
    for (auto ctx : connections_)
      ctx->data_to_send.Update(data.c_str(), data.length());

    MHD_run(d_);
    return true;
  }

private:
  static MHD_Result AccessHandlerCallbackWrapper(
    void *cls, struct MHD_Connection *connection,
    const char *url, const char *method, const char *version,
    const char *upload_data, size_t *upload_data_size, void **ptr) {
    MHDServer *server = (MHDServer *)cls;
    return server->AccessHandlerCallback(
      connection, url, method, version,
      upload_data, upload_data_size, ptr);
  }

  MHD_Result AccessHandlerCallback(
    struct MHD_Connection *connection,
    const char *url, const char *method, const char *version,
    const char *upload_data, size_t *upload_data_size, void **ptr);

  static void NotifyConnectionCallbackWrapper(
    void *cls, struct MHD_Connection *connection,
    void **socket_context, enum MHD_ConnectionNotificationCode toe) {
    MHDServer *server = (MHDServer *)cls;
    return server->NotifyConnectionCallback(connection, socket_context, toe);
  }

  struct connection_context {
    OutputBuffer<char> data_to_send;
  };
  std::set<struct connection_context *> connections_;

  void NotifyConnectionCallback(
    struct MHD_Connection *connection, void **socket_context,
    enum MHD_ConnectionNotificationCode toe) {
    switch (toe) {
    case MHD_CONNECTION_NOTIFY_STARTED: {
      struct connection_context *context = new connection_context();
      connections_.insert(context);
      *socket_context = (void *) context;
      break;
    }
    case MHD_CONNECTION_NOTIFY_CLOSED: {
      struct connection_context *context =
        (struct connection_context *) *socket_context;
      connections_.erase(context);
      delete context;
      break;
    }
    default: break;
    }
  }

  struct MHD_Daemon *d_;
};

static ssize_t data_generator (void *cls, uint64_t pos, char *buf, size_t max) {
  OutputBuffer<char> *data = (OutputBuffer<char> *) cls;
  return data->Consume(buf, max);
}

MHD_Result MHDServer::AccessHandlerCallback(
  struct MHD_Connection *connection,
  const char *url, const char *method, const char *version,
  const char *upload_data, size_t *upload_data_size, void **ptr) {
  static int aptr;
  if (0 != strcmp (method, "GET"))
    return MHD_NO;

  // Do not respond on first call
  if (&aptr != *ptr) {
    *ptr = &aptr;
    return MHD_YES;
  }

  *ptr = NULL;
  if (strcmp (url, "/api/stream") != 0) {
    return MHD_NO;
  }

  struct connection_context *context =
    *(struct connection_context **) MHD_get_connection_info(
      connection, MHD_CONNECTION_INFO_SOCKET_CONTEXT);
  struct MHD_Response *response = NULL;
  unsigned int status_code = MHD_HTTP_OK;
  if (context->data_to_send.size())
    response = MHD_create_response_from_callback(
      MHD_SIZE_UNKNOWN, 1024, &data_generator, &context->data_to_send, NULL);
  else
    status_code = MHD_HTTP_NO_CONTENT;

  MHD_queue_response(connection, status_code, response);
  if (response)
    MHD_destroy_response(response);

  return MHD_YES;
}

class StatusServer::Impl : public GCodeMachineControl::StatusEventReceiver {
public:
  Impl() : machine_control_(NULL), server_(NULL) {}
  virtual ~Impl() {}
  bool Init(FDMultiplexer *event_server, const char *bind_addr, int port);

  virtual void GCodeMachineControlCreated(GCodeMachineControl *machine_control) {
    machine_control_ = machine_control;
  }

  virtual void enqueued_line_move(float feed_mm_p_sec, const AxesRegister &absolute_pos) {
    std::stringstream ss;
    ss << "{ ";
    ss << "\"feedrate\": " << feed_mm_p_sec << ", ";
    ss << "\"absolute_pos\": [ ";
    for (auto it = absolute_pos.begin();
         it != absolute_pos.end(); ++it) {
      ss << *it;
      if (it != absolute_pos.end() - 1)
        ss << ", ";
    }
    ss << " ]";
    ss << " }\n";
    const std::string data(ss.str());
    server_->SendData(data);
  }

  virtual void enqueued_arc_move(float feed_mm_p_sec,
                                 GCodeParserAxis normal_axis, bool clockwise,
                                 const AxesRegister &start,
                                 const AxesRegister &center,
                                 const AxesRegister &end) {}
  virtual void enqueued_spline_move(float feed_mm_p_sec,
                                    const AxesRegister &start,
                                    const AxesRegister &cp1, const AxesRegister &cp2,
                                    const AxesRegister &end) {}
  virtual void paused() {}
  virtual void resumed() {}

private:
  GCodeMachineControl *machine_control_;
  MHDServer *server_;
};

bool StatusServer::Impl::Init(FDMultiplexer *event_server, const char *bind_addr, int port) {
  server_ = new MHDServer(port);
  event_server->AddExternalHandler(server_);
  return true;
}

StatusServer::~StatusServer() = default;
StatusServer::StatusServer(Impl *impl) : impl_(impl) {}

GCodeMachineControl::StatusEventReceiver *StatusServer::StatusEventReceiver() {
  return impl_;
}

StatusServer *StatusServer::Create(FDMultiplexer *event_server,
                                   const char *bind_addr, int port) {
  StatusServer::Impl *impl = new StatusServer::Impl();
  if (!impl->Init(event_server, bind_addr, port)) {
    delete impl;
    return NULL;
  }

  return new StatusServer(impl);
}
