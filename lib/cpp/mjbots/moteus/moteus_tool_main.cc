// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "utils/moteus_tool.h"

#include <memory>
#include <thread>

#include <boost/asio/executor.hpp>
#include <boost/asio/post.hpp>

#include "mjlib/base/fail.h"
#include "mjlib/io/deadline_timer.h"
#include "mjlib/io/selector.h"
#include "mjlib/multiplex/asio_client.h"

#include "mjbots/pi3hat/pi3hat.h"

namespace {
template <typename T>
uint32_t u32(T value) {
  return static_cast<uint32_t>(value);
}

template <typename T, typename InType>
T Saturate(InType value) {
  if (value >= static_cast<InType>(std::numeric_limits<T>::max())) {
    return std::numeric_limits<T>::max();
  }
  if (value <= static_cast<InType>(std::numeric_limits<T>::min())) {
    return std::numeric_limits<T>::min();
  }
  return static_cast<T>(value);
}

class Pi3hatWrapper : public mjlib::multiplex::AsioClient {
 public:
  struct Options {
    int spi_speed_hz = 10000000;
    double query_timeout_s = 0.001;
    int force_bus = -1;

    // And guarantee to wait at least this long after any successful
    // receives to catch stragglers. (Mostly to prevent stale data
    // from the previous cycle causing us to be one cycle behind).
    double min_wait_s = 0.00005;


    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(spi_speed_hz));
      a->Visit(MJ_NVP(query_timeout_s));
      a->Visit(MJ_NVP(force_bus));
      a->Visit(MJ_NVP(min_wait_s));
    }
  };

  Pi3hatWrapper(const boost::asio::any_io_executor& executor,
                const Options& options)
      : executor_(executor),
        options_(options) {
    thread_ = std::thread(std::bind(&Pi3hatWrapper::CHILD_Run, this));
  }

  ~Pi3hatWrapper() {
    child_context_.stop();
    thread_.join();
  }

  void AsyncStart(mjlib::io::ErrorCallback callback) {
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));

  }

  void AsyncTransmit(const AsioClient::Request* request,
                     Reply* reply,
                     mjlib::io::ErrorCallback callback) override {
    boost::asio::post(
        child_context_,
        [this, callback=std::move(callback), request, reply]() mutable {
          this->CHILD_Transmit(request, reply, std::move(callback));
        });
  }

  mjlib::io::SharedStream MakeTunnel(
      uint8_t id,
      uint32_t channel,
      const TunnelOptions& options) override {
    return std::make_shared<Tunnel>(this, id, channel, options);
  }

  class Tunnel : public mjlib::io::AsyncStream,
                 public std::enable_shared_from_this<Tunnel> {
   public:
    Tunnel(Pi3hatWrapper* parent, uint8_t id, uint32_t channel,
           const TunnelOptions& options)
        : parent_(parent),
          id_(id),
          channel_(channel),
          options_(options) {}

    ~Tunnel() override {}

    void async_read_some(mjlib::io::MutableBufferSequence buffers,
                         mjlib::io::ReadHandler handler) override {
      boost::asio::post(
          parent_->child_context_,
          [self=shared_from_this(), buffers,
           handler=std::move(handler)]() mutable {
            const auto bytes_read = self->parent_->CHILD_TunnelPoll(
                self->id_, self->channel_, buffers);
            if (bytes_read > 0) {
              boost::asio::post(
                  self->parent_->executor_,
                  [self, handler=std::move(handler), bytes_read]() mutable {
                    handler(mjlib::base::error_code(), bytes_read);
                  });
            } else {
              self->timer_.expires_from_now(self->options_.poll_rate);
              self->timer_.async_wait(
                  [self, handler=std::move(handler), buffers](
                      const auto& ec) mutable {
                    self->HandlePoll(ec, buffers, std::move(handler));
                  });
            }
          });
    }

    void async_write_some(mjlib::io::ConstBufferSequence buffers,
                          mjlib::io::WriteHandler handler) override {
      boost::asio::post(
          parent_->child_context_,
          [self=shared_from_this(), buffers, handler=std::move(handler)]() mutable {
            self->parent_->CHILD_TunnelWrite(
                self->id_, self->channel_, buffers, std::move(handler));
          });
    }

    boost::asio::any_io_executor get_executor() override {
      return parent_->executor_;
    }

    void cancel() override {
      timer_.cancel();
    }

   private:
    void HandlePoll(const mjlib::base::error_code& ec,
                    mjlib::io::MutableBufferSequence buffers,
                    mjlib::io::ReadHandler handler) {
      if (ec) {
        handler(ec, 0);
        return;
      }

      async_read_some(buffers, std::move(handler));
    }

    Pi3hatWrapper* const parent_;
    const uint8_t id_;
    const uint32_t channel_;
    const TunnelOptions options_;

    mjlib::io::DeadlineTimer timer_{parent_->executor_};
  };

  void CHILD_Run() {
    pi3hat_.emplace([&]() {
        mjbots::pi3hat::Pi3Hat::Configuration c;
        c.spi_speed_hz = options_.spi_speed_hz;
        return c;
      }());

    boost::asio::io_context::work work{child_context_};
    child_context_.run();

    // Destroy before we finish in the child thread.
    pi3hat_.reset();
  }

  void CHILD_SetupCAN(mjbots::pi3hat::Pi3Hat::Input* input,
                      const Request* requests) {
    auto& d = pi3data_;
    d.tx_can.clear();

    for (const auto& request : *requests) {
      d.tx_can.push_back({});
      auto& dst = d.tx_can.back();
      dst.id = request.id | (request.request.request_reply() ? 0x8000 : 0x00);
      dst.size = request.request.buffer().size();
      std::memcpy(&dst.data[0], request.request.buffer().data(), dst.size);
      dst.bus = SelectBus(request.id);
      dst.expect_reply = request.request.request_reply();
    }

    if (d.tx_can.size()) {
      input->tx_can = {&d.tx_can[0], d.tx_can.size()};
    }

    d.rx_can.resize(std::max<size_t>(d.tx_can.size() * 2, 24));
    input->rx_can = {&d.rx_can[0], d.rx_can.size()};
  }

  void CHILD_Transmit(const Request* request,
                      Reply* reply,
                      mjlib::io::ErrorCallback callback) {
    mjbots::pi3hat::Pi3Hat::Input input;

    CHILD_SetupCAN(&input, request);

    input.timeout_ns = options_.query_timeout_s * 1e9;

    pi3data_.result = pi3hat_->Cycle(input);

    // Now come back to the main thread.
    boost::asio::post(
        executor_,
        [this, callback=std::move(callback), reply]() mutable {
          this->FinishTransmit(reply, std::move(callback));
        });
  }

  size_t CHILD_TunnelPoll(uint8_t id, uint32_t channel,
                          mjlib::io::MutableBufferSequence buffers) {
    mjbots::pi3hat::Pi3Hat::Input input;

    if (pi3data_.rx_can.size() < 4) {
      pi3data_.rx_can.resize(4);
    }

    for (auto& frame : pi3data_.rx_can) {
      std::memset(&frame.data[0], 0, sizeof(frame.data));
    }
    input.rx_can = {&pi3data_.rx_can[0], pi3data_.rx_can.size()};
    input.force_can_check = (1 << SelectBus(id));
    input.timeout_ns = 0;
    input.min_tx_wait_ns = 0;

    // Check for anything lying around first.
    pi3data_.result = pi3hat_->Cycle(input);

    if (pi3data_.result.rx_can_size > 0) {
      return CHILD_ParseTunnelPoll(id, channel, buffers);
    }

    input.timeout_ns = options_.query_timeout_s * 1e9;
    input.min_tx_wait_ns = options_.min_wait_s * 1e9;

    // Nope, so we should poll.
    if (pi3data_.tx_can.size() < 1) {
      pi3data_.tx_can.resize(1);
    }

    auto& out_frame = pi3data_.tx_can[0];
    mjlib::base::BufferWriteStream stream{
      {reinterpret_cast<char*>(&out_frame.data[0]), sizeof(out_frame.data)}};
    mjlib::multiplex::WriteStream writer{stream};

    writer.WriteVaruint(
        u32(mjlib::multiplex::Format::Subframe::kClientPollServer));
    writer.WriteVaruint(channel);
    writer.WriteVaruint(
        std::min<uint32_t>(48, boost::asio::buffer_size(buffers)));

    out_frame.expect_reply = true;
    out_frame.bus = SelectBus(id);
    out_frame.id = 0x8000 | id;
    out_frame.size = stream.offset();

    input.tx_can = {&pi3data_.tx_can[0], 1};

    pi3data_.result = pi3hat_->Cycle(input);

    return CHILD_ParseTunnelPoll(id, channel, buffers);
  }

  size_t CHILD_ParseTunnelPoll(uint8_t id, uint32_t channel,
                               mjlib::io::MutableBufferSequence buffers) {
    size_t result = 0;

    for (size_t i = 0; i < pi3data_.result.rx_can_size; i++) {
      const auto& src = pi3data_.rx_can[i];
      if (((src.id >> 8) & 0xff) != id) { continue; }

      mjlib::base::BufferReadStream buffer_stream{
        {reinterpret_cast<const char*>(&src.data[0]), src.size}};
      mjlib::multiplex::ReadStream<
        mjlib::base::BufferReadStream> stream{buffer_stream};

      const auto maybe_subframe = stream.ReadVaruint();
      if (!maybe_subframe || *maybe_subframe !=
          u32(mjlib::multiplex::Format::Subframe::kServerToClient)) {
        continue;
      }

      const auto maybe_channel = stream.ReadVaruint();
      if (!maybe_channel || *maybe_channel != channel) {
        continue;
      }

      const auto maybe_stream_size = stream.ReadVaruint();
      if (!maybe_stream_size) {
        continue;
      }

      const auto stream_size = *maybe_stream_size;
      if (stream_size == 0) { continue; }

      auto remaining_data = stream_size;
      for (auto buffer : buffers) {
        if (remaining_data == 0) { break; }

        const auto to_read = std::min<size_t>(buffer.size(), remaining_data);
        buffer_stream.read({static_cast<char*>(buffer.data()),
                static_cast<std::streamsize>(to_read)});
        remaining_data -= to_read;
        result += to_read;
      }
    }

    return result;
  }

  void CHILD_TunnelWrite(uint8_t id, uint32_t channel,
                         mjlib::io::ConstBufferSequence buffers,
                         mjlib::io::WriteHandler callback) {
    if (pi3data_.tx_can.size() < 1) {
      pi3data_.tx_can.resize(1);
    }
    auto& dst = pi3data_.tx_can[0];

    mjlib::base::BufferWriteStream stream{
      {reinterpret_cast<char*>(&dst.data[0]), sizeof(dst.data)}};
    mjlib::multiplex::WriteStream writer{stream};

    writer.WriteVaruint(u32(mjlib::multiplex::Format::Subframe::kClientToServer));
    writer.WriteVaruint(channel);

    const auto size = std::min<size_t>(48, boost::asio::buffer_size(buffers));
    writer.WriteVaruint(size);
    auto remaining_size = size;
    for (auto buffer : buffers) {
      if (remaining_size == 0) { break; }
      const auto to_write = std::min(remaining_size, buffer.size());
      stream.write({static_cast<const char*>(buffer.data()), to_write});
      remaining_size -= to_write;
    }

    dst.id = id;
    dst.bus = SelectBus(id);
    dst.size = stream.offset();

    mjbots::pi3hat::Pi3Hat::Input input;
    input.tx_can = {&pi3data_.tx_can[0], 1};

    pi3hat_->Cycle(input);

    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code(), size));
  }

  void FinishCAN(Reply* reply) {
    // First CAN.
    for (size_t i = 0; i < pi3data_.result.rx_can_size; i++) {
      const auto& src = pi3data_.rx_can[i];

      if (src.id == 0x10004) {
        // This came from the power_dist board.  Ignore.
        continue;
      }

      parsed_data_.clear();
      mjlib::base::BufferReadStream payload_stream{
        {reinterpret_cast<const char*>(&src.data[0]),
              pi3data_.rx_can[i].size}};
      mjlib::multiplex::ParseRegisterReply(payload_stream, &parsed_data_);
      for (const auto& pair : parsed_data_) {
        reply->push_back({static_cast<uint8_t>((src.id >> 8) & 0xff),
                pair.first, pair.second});
      }
    }
  }

  void FinishTransmit(Reply* reply, mjlib::io::ErrorCallback callback) {
    FinishCAN(reply);

    // Finally, post our CAN response.
    boost::asio::post(
        executor_,
        std::bind(std::move(callback), mjlib::base::error_code()));
  }

  int SelectBus(int id) const {
    if (options_.force_bus >= 0) { return options_.force_bus; }

    return (id >= 1 && id <= 3) ? 1 :
        (id >= 4 && id <= 6) ? 2 :
        (id >= 7 && id <= 9) ? 3 :
        (id >= 10 && id <= 12) ? 4 :
        1; // just send everything else out to 1 by default
  }

 private:
  boost::asio::any_io_executor executor_;
  const Options options_;

  std::thread thread_;

  std::vector<mjlib::multiplex::RegisterValue> parsed_data_;

  // Only accessed from the thread.
  std::optional<mjbots::pi3hat::Pi3Hat> pi3hat_;
  boost::asio::io_context child_context_;

  // This can be accessed in either thread, but never at the same
  // time.  They can either be accessed inside CHILD_Register, or in
  // the parent until the callback is invoked.
  struct Pi3Data {
    std::vector<mjbots::pi3hat::CanFrame> tx_can;
    std::vector<mjbots::pi3hat::CanFrame> rx_can;

    mjbots::pi3hat::Pi3Hat::Output result;
  };
  Pi3Data pi3data_;
};
}

int main(int argc, char** argv) {
  boost::asio::io_context context;
  mjlib::io::Selector<mjlib::multiplex::AsioClient> client_selector{
    context.get_executor(), "client_type"};
  client_selector.Register<Pi3hatWrapper>("pi3");
  client_selector.set_default("pi3");
  return moteus::tool::moteus_tool_main(context, argc, argv, &client_selector);
}
