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

#include "rclcpp/rclcpp.hpp"

#include "config_utilities/config.h"              // Enables declare_config().
#include "config_utilities/formatting/asl.h"      // Simply including this file sets a style to format output.
//TODO: should put into single ros header and use #defines to determine which one to include?
#include "config_utilities/logging/log_to_ros2.h"  // Simply including this file sets logging to roslog.
#include "config_utilities/parsing/ros2.h"         // Enable fromRos().
#include "config_utilities/printing.h"            // Enable toString()
#include "config_utilities/traits.h"              // Enables isConfig()
#include "config_utilities/types/eigen_matrix.h"  // Enable parsing and printing of Eigen::Matrix types.
#include "config_utilities/types/enum.h"          // Enable parsing and printing of enum types.
#include "config_utilities/validation.h"          // Enable isValid() and checkValid().


namespace demo {

// A sub-struct for later use.
struct SubConfig {
  float f = 0.123;
  std::string s = "test";
};

// A struct that represents what we want to be a config.
// Requirements for a config struct: is default constructable.
struct MyConfig {
  int i = 100;
  double distance = 42;
  bool b = true;
  std::vector<int> vec = {1, 2, 3};
  std::map<std::string, int> map = {{"a", 1}, {"b", 2}, {"c", 3}};
  Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Identity();
  enum class MyEnum { kA, kB, kC } my_enum = MyEnum::kA;
  SubConfig sub_config;
};

// Defining 'void declare_config(T& config)' function labels a struct as config.
// It **MUST** be declared beforehand if being used in another declare_config
void declare_config(SubConfig&);

// All config properties are specified within declare_config.
void declare_config(MyConfig& config) {
  config::name("MyConfig");
  config::field(config.i, "i");
  config::field(config.distance, "distance", "m");
  config::field(config.b, "b");
  config::field(config.vec, "vec");
  config::field(config.map, "map");
  config::field(config.mat, "mat");
  config::enum_field(config.my_enum, "my_enum", {"A", "B", "C"});
  config::NameSpace ns("sub_ns");
  config::field(config.sub_config, "sub_config");

  config::check(config.i, config::CheckMode::GT, 0, "i");
}

// Declaration of the subconfig.
void declare_config(SubConfig& config) {
  using namespace config;
  name("SubConfig");
  field(config.f, "f");
  field(config.s, "s");
  check(config.f, CheckMode::GT, 0.f, "f");
}

// Declare objects with configs to create from the factory.

class Base {
 public:
  virtual void print() const = 0;
  virtual ~Base() = default;
};

class DerivedA : public Base {
 public:
  struct Config {
    float f = 0.f;
  };

  explicit DerivedA(const Config& config) : config_(config) { config::checkValid(config_); }

  void print() const override { std::cout << "I'm a DerivedA with config.f='" << config_.f << "'."; }

 private:
  const Config config_;

  // Register the module to the factory with a static registration struct. Signature:
  // RegistrationWithConfig<BaseT, DerivedT, DerivedConfigT, ConstructorArguments...>(string identifier).
  inline static const auto registration_ = config::RegistrationWithConfig<Base, DerivedA, DerivedA::Config>("DerivedA");
};

void declare_config(DerivedA::Config& config) {
  // Declare the config using the config utilities.
  config::name("DerivedA");
  config::field(config.f, "f");
  config::check(config.f, config::CheckMode::GE, 0.f, "f");
}

struct RosConfig {
    int parameter_a;
    int parameter_b;
};

void declare_config(RosConfig& config) {
    config::name("RosConfig");
    config::field(config.parameter_a, "parameter_a");
    config::field(config.parameter_b, "parameter_b");
}

} //demo


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions no;
//   no.parameter_overrides(
//   {
//     {"f", 42.0},
//     // {"parameter_no_default_set", 42},
//   });
  auto node = std::make_shared<rclcpp::Node>("test_declare_parameter_node", no);
//   node->declare_parameter("f", 43.0);
// //   node->declare_parameter("sub_ns.parameter_no_default", 43);

// //  auto config = config::internal::rosToYaml(node);
// //  config = config::internal::rosToYaml(node->create_sub_node("sub_ns"));
// auto config = config::fromRos<demo::DerivedA::Config>(node);

auto values = node->declare_parameters<int64_t>(
      "namespace1", {
      {"parameter_a", 42},
      {"parameter_b", 256},
    });

    node->declare_parameters<int64_t>(
        "", { 
        {"parameter_a", 10}, 
        {"parameter_b", 12} 
        }
    );

 auto config = config::fromRos<demo::RosConfig>(node, "");
 auto config1 = config::fromRos<demo::RosConfig>(node, "namespace1");

std::cout << config <<std::endl;
std::cout << config1 <<std::endl;

 rclcpp::spin(node);

}