/*
 *   Copyright (c) 2024 ACFR-RPG, University of Sydney, Jesse Morris (jesse.morris@sydney.edu.au)
 *   All rights reserved.

 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

#pragma once

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "config_utilities/factory.h"
#include "config_utilities/internal/logger.h"

namespace config::internal {

/**
 * @brief Implements logging to roslog. This file pulls in ros as a dependency, but its not required if this file is
 * not included in the project.
 */
class Ros2Logger : public Logger {
 public:
  Ros2Logger() = default;
  virtual ~Ros2Logger() = default;

 protected:
    //TODO: debug?
  void logImpl(const Severity severity, const std::string& message) override {
    switch (severity) {
      case Severity::kInfo:
        RCLCPP_INFO_STREAM(logger_, message);
        break;

      case Severity::kWarning:
        RCLCPP_WARN_STREAM(logger_, message);
        break;

      case Severity::kError:
        RCLCPP_ERROR_STREAM(logger_, message);
        break;

      case Severity::kFatal:
        RCLCPP_FATAL_STREAM(logger_, message);
    }
  }

 private:
  // Factory registration to allow setting of formatters via Settings::setLogger().
  inline static const auto registration_ = Registration<Logger, Ros2Logger>("ros");
  // Use a logger directly instead of a node
  //TODO: how to select name?
  rclcpp::Logger logger_ = rclcpp::get_logger("ros2");

  // Initialize the ros logger to be used if included.
  inline static const struct Initializer {
    Initializer() { Logger::setLogger(std::make_shared<Ros2Logger>()); }
  } initializer_;
};

}  // namespace config::internal
