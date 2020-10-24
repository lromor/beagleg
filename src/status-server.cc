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
#include "common/container.h"

template<typename T, size_t CHUNK_SIZE = 4096>
class OutputBuffer {
public:
  OutputBuffer() : back_write_pos_(CHUNK_SIZE), front_read_pos_(CHUNK_SIZE) {}

  void Update(const T *src, const size_t size) {
    T *back = NULL;
    size_t src_pos = 0;

    while (src_pos < size) {
      if (back_write_pos_ == CHUNK_SIZE) {
        back = new T[CHUNK_SIZE];
        if (data_.empty())
          front_read_pos_ = 0;
        data_.push_back(std::unique_ptr<T>(back));
        back_write_pos_ = 0;
      } else
        back = data_.back().get();

      // Fill the chunk
      const size_t available_chunk_size = CHUNK_SIZE - back_write_pos_;
      const size_t remaining = size - src_pos;
      const size_t copy_size =
        (available_chunk_size > remaining)
        ? remaining : available_chunk_size;
      memcpy(back + back_write_pos_, src + src_pos, copy_size * sizeof(T));
      src_pos += copy_size;
      back_write_pos_ += copy_size;
    }
  }

  const size_t Consume(void *dst, const size_t size_bytes) {
    const size_t size = size_bytes / sizeof(T);
    T *dst_t = (T *)dst;
    size_t dst_pos = 0;
    T *front = NULL;

    while (dst_pos < size) {
      if (data_.empty() || (data_.size() == 1 && (front_read_pos_ == back_write_pos_)))
        break;

      if (front_read_pos_ == CHUNK_SIZE) {
        data_.pop_front();
        front_read_pos_ = 0;
      }
      front = data_.front().get();

      const size_t available_chunk_size = (data_.size() == 1)
        ? back_write_pos_ - front_read_pos_
        : CHUNK_SIZE - front_read_pos_;
      const size_t remaining = size - dst_pos;
      const size_t copy_size =
        (available_chunk_size > remaining)
        ? remaining : available_chunk_size;
      memcpy(dst_t + dst_pos, front + front_read_pos_, copy_size * sizeof(T));
      dst_pos += copy_size;
      front_read_pos_ += copy_size;
    }
    return dst_pos * sizeof(T);
  }

  const size_t empty() {
    if (!data_.size()) {
      return true;
    }
    const bool same_pos =
      front_read_pos_ != back_write_pos_;
    if (data_.size() == 1
        && front_read_pos_ == back_write_pos_) {
      return true;
    }
    return false;
  }

private:
  std::deque<std::unique_ptr<T>> data_;
  size_t back_write_pos_;
  size_t front_read_pos_;
};

static ssize_t data_generator (void *cls, uint64_t pos, char *buf, size_t max) {
  OutputBuffer<char> *data = (OutputBuffer<char> *) cls;
  return data->Consume(buf, max);
}

class MHDServer : public FDMultiplexer::ExternalHandler {
public:
  MHDServer(const int port) : d_(NULL) {
    d_ = MHD_start_daemon(
      // MHD_USE_DEBUG |
      MHD_ALLOW_SUSPEND_RESUME,
      port, NULL, NULL,
      &AccessHandlerCallbackWrapper, (void *) this,
      MHD_OPTION_NOTIFY_CONNECTION, &NotifyConnectionCallbackWrapper, (void *) this,
      MHD_OPTION_CONNECTION_TIMEOUT, (unsigned int) 10,
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
    MHD_Connection *connection;
  };
  std::set<struct connection_context *> connections_;

  void NotifyConnectionCallback(
    struct MHD_Connection *connection, void **socket_context,
    enum MHD_ConnectionNotificationCode toe) {
    switch (toe) {
    case MHD_CONNECTION_NOTIFY_STARTED: {
      struct connection_context *context =
        new connection_context{ {}, connection};
      connections_.insert(context);
      *socket_context = (void *) context;
      Log_info("[Status server] Accepting new connection. (Total connections: %ld)",
               connections_.size());
      break;
    }
    case MHD_CONNECTION_NOTIFY_CLOSED: {
      struct connection_context *context =
        (struct connection_context *) *socket_context;
      connections_.erase(context);
      delete context;
      Log_info("[Status server] Connection closed.(Total connections: %ld)",
               connections_.size());
      MHD_run(d_);
      break;
    }
    default: break;
    }
  }

  struct MHD_Daemon *d_;
};

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
  if (!context->data_to_send.empty()) {
    struct MHD_Response *response = MHD_create_response_from_callback(
      MHD_SIZE_UNKNOWN, 4096, &data_generator, &context->data_to_send, NULL);

    MHD_queue_response(connection, MHD_HTTP_OK, response);
    MHD_destroy_response(response);
  }
  return MHD_YES;
}

class StatusServer::Impl : public GCodeMachineControl::StatusEventReceiver {
public:
  Impl() : machine_control_(NULL), server_(NULL) {}
  virtual ~Impl() {}
  bool Init(FDMultiplexer *event_server, const char *bind_addr, int port);

  std::string GetStatus() {
    std::stringstream ss;
    ss << "{ ";
    if (machine_control_) {
      AxesRegister pos;
      machine_control_->GetCurrentPosition(&pos);
      ss << "\"current_pos\": [ ";
      for (auto it = pos.begin();
           it != pos.end(); ++it) {
        ss << *it;
        if (it != pos.end() - 1)
          ss << ", ";
      }
      ss << " ]";
    }
    ss << " }\n";
    return ss.str();
  }

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
  event_server->RunOnIdle([&]() {
    server_->SendData(GetStatus());
    return true;
  });
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
