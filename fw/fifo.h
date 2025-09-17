// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#pragma once

#include <atomic>
#include <cstdlib>

namespace fw {

// Atomic fifo for non-movable/copyable types.
template<typename T, size_t size>
class AtomicFifo {
 public:
  bool full() const {
    // We rely on unsigned wraparound to correctly calculate the size.
    return static_cast<uint32_t>(put_index_.load() - get_index_.load()) == size;
  }

  bool empty() const {
    return put_index_.load() == get_index_.load();
  }

  T* prepare_push() {
    if (full()) { return nullptr; }

    const auto put_index = put_index_.load();

    return &data_[put_index % size];
  }

  void push() {
    const auto put_index = put_index_.load();
    put_index_.store(put_index + 1);
  }


  T* prepare_get() {
    const auto put_index = put_index_.load();
    const auto get_index = get_index_.load();
    if (put_index == get_index) { return nullptr; }
    return &data_[get_index % size];
  }

  void get() {
    const auto get_index = get_index_.load();
    get_index_.store(get_index + 1);
  }

  T* operator()(size_t i) {
    const auto get_index = get_index_.load();
    const auto put_index = put_index_.load();
    const auto amount = static_cast<uint32_t>(put_index - get_index);
    if (i >= amount) { return nullptr; }
    return &data_[(get_index + i) % size];
  }

 private:
  T data_[size] = {};

  std::atomic<uint32_t> get_index_{0};
  std::atomic<uint32_t> put_index_{0};
};
}
