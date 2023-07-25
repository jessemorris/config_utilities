#include "config_utilities/virtual_config.h"

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/formatting/asl.h"
#include "config_utilities/logging/log_to_stdout.h"
#include "config_utilities/parsing/yaml.h"
#include "config_utilities/printing.h"
#include "config_utilities/test/utils.h"
#include "config_utilities/validation.h"

namespace config::test {

class Base2 {
 public:
  virtual std::string name() const = 0;
};

class Derived2 : public Base2 {
 public:
  struct Config {
    float f = 0.f;
    std::string s = "test string";
  };
  explicit Derived2(const Config& config) : config_(config) {}
  std::string name() const override { return "Derived2"; }
  const Config config_;
  inline static const auto registration_ =
      config::RegistrationWithConfig<Base2, Derived2, Derived2::Config>("Derived2");
};

void declare_config(Derived2::Config& config) {
  // Declare the config using the config utilities.
  config::name("Derived2");
  config::field(config.f, "f", "m/s");
  config::field(config.s, "s");
  config::check(config.f, config::CheckMode::GE, 0.f, "f");
}

class ObjectWithBase {
 public:
  struct Config {
    double d = 0.0;
    VirtualConfig<Base2> base_config;
  };
  explicit ObjectWithBase(const Config& config) : config_(config) { base_ = config_.base_config.create(); }
  const Config config_;
  std::unique_ptr<Base2> base_;
};

void declare_config(ObjectWithBase::Config& config) {
  // Declare the config using the config utilities.
  config::name("ObjectWithBase");
  config::field(config.d, "d", "kg/m^3");
  config::check(config.d, config::CheckMode::GE, 0.0, "d");
  config::field(config.base_config, "base_config");
}

TEST(VirtualConfig, isSet) {
  Settings().restoreDefaults();

  VirtualConfig<Base2> config;
  EXPECT_FALSE(config.isSet());
  EXPECT_FALSE(isValid(config));

  YAML::Node data;
  config = fromYaml<VirtualConfig<Base2>>(data);
  EXPECT_FALSE(config.isSet());
  EXPECT_FALSE(isValid(config));

  data["type"] = "Derived2";
  config = fromYaml<VirtualConfig<Base2>>(data);
  EXPECT_TRUE(config.isSet());
  EXPECT_TRUE(isValid(config));
  EXPECT_EQ(config.getType(), "Derived2");
}

TEST(VirtualConfig, copyMove) {
  VirtualConfig<Base2> config;
  VirtualConfig<Base2> other;

  config = other;
  EXPECT_FALSE(config.isSet());

  config = std::move(other);
  EXPECT_FALSE(config.isSet());

  VirtualConfig<Base2> config2(config);
  EXPECT_FALSE(config2.isSet());

  VirtualConfig<Base2> config3(std::move(config));
  EXPECT_FALSE(config3.isSet());

  YAML::Node data;
  data["type"] = "Derived2";
  other = fromYaml<VirtualConfig<Base2>>(data);

  config = other;
  EXPECT_TRUE(config.isSet());
  EXPECT_EQ(config.getType(), "Derived2");

  config = std::move(other);
  EXPECT_TRUE(config.isSet());
  EXPECT_EQ(config.getType(), "Derived2");

  VirtualConfig<Base2> config4(config);
  EXPECT_TRUE(config4.isSet());
  EXPECT_EQ(config4.getType(), "Derived2");

  VirtualConfig<Base2> config5(std::move(config));
  EXPECT_TRUE(config5.isSet());
  EXPECT_EQ(config5.getType(), "Derived2");
}

TEST(VirtualConfig, create) {
  VirtualConfig<Base2> config;
  std::unique_ptr<Base2> object = config.create();
  EXPECT_FALSE(object);

  YAML::Node data;
  data["type"] = "Derived2";
  data["f"] = 1.f;
  config = fromYaml<VirtualConfig<Base2>>(data);
  object = config.create();
  EXPECT_TRUE(object);
  EXPECT_EQ(object->name(), "Derived2");
  EXPECT_EQ(dynamic_cast<Derived2*>(object.get())->config_.f, 1.f);
}

TEST(VirtualConfig, isOptional) {
  VirtualConfig<Base2> config;
  auto logger = TestLogger::create();
  EXPECT_FALSE(isValid(config, true));
  EXPECT_EQ(logger->numMessages(), 1);
  std::string msg = logger->messages()[0].second;
  std::string expected = R"""(Invalid config 'Uninitialized Virtual Config':
========================= Uninitialized Virtual Config =========================
Warning: Virtual config is not set and not marked optional.
================================================================================)""";
  EXPECT_EQ(msg, expected);

  config.setOptional();
  EXPECT_TRUE(isValid(config));
}

TEST(VirtualConfig, printing) {
  VirtualConfig<Base2> config;
  std::string msg = toString(config);
  std::string expected = R"""(========================= Uninitialized Virtual Config =========================
================================================================================)""";
  EXPECT_EQ(msg, expected);

  YAML::Node data;
  data["type"] = "Derived2";
  data["f"] = 1.f;
  data["s"] = "test string";
  config = fromYaml<VirtualConfig<Base2>>(data);

  EXPECT_TRUE(isValid(config));
  msg = toString(config);
  expected = R"""(=========================== Virtual Config: Derived2 ===========================
f [m/s]:                      1
s:                            test string (default)
================================================================================)""";
  EXPECT_EQ(msg, expected);

  auto logger = TestLogger::create();
  data["f"] = -1.f;
  config = fromYaml<VirtualConfig<Base2>>(data);
  EXPECT_FALSE(isValid(config, true));
  EXPECT_EQ(logger->numMessages(), 1);
  msg = logger->messages()[0].second;
  expected = R"""(Invalid config 'Virtual Config: Derived2':
=========================== Virtual Config: Derived2 ===========================
Warning: Check failed for 'f': param >= 0 (is: '-1').
================================================================================)""";
  EXPECT_EQ(msg, expected);
}

TEST(VirtualConfig, subconfig) {
  ObjectWithBase::Config config;
  config.d = 1.0;
  std::string msg = toString(config);
  std::string expected = R"""(================================ ObjectWithBase ================================
d [kg/m^3]:                   1
base_config [Uninitialized Virtual Config]
================================================================================)""";
  EXPECT_EQ(msg, expected);
  EXPECT_FALSE(isValid(config));

  YAML::Node data;
  data["d"] = 1.0;
  data["type"] = "Derived2";
  data["f"] = 1.f;
  config = fromYaml<ObjectWithBase::Config>(data);
  EXPECT_TRUE(isValid(config));
  EXPECT_EQ(config.base_config.getType(), "Derived2");

  msg = toString(config);
  expected = R"""(================================ ObjectWithBase ================================
d [kg/m^3]:                   1
base_config [Virtual Config: Derived2]:
   f [m/s]:                   1
   s:                         test string (default)
================================================================================)""";
  EXPECT_EQ(msg, expected);

  data["f"] = -1.f;
  config = fromYaml<ObjectWithBase::Config>(data);
  auto logger = TestLogger::create();
  EXPECT_FALSE(isValid(config, true));
  EXPECT_EQ(logger->numMessages(), 1);
  msg = logger->messages()[0].second;
  expected = R"""(Invalid config 'ObjectWithBase':
================================ ObjectWithBase ================================
Warning: Check failed for 'base_config.f': param >= 0 (is: '-1').
================================================================================)""";
  EXPECT_EQ(msg, expected);
}

}  // namespace config::test
