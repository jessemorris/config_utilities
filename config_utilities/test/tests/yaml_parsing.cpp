#include <sstream>

#include <gtest/gtest.h>

#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/parsing/yaml.h"
#include "config_utilities/test/default_config.h"
#include "config_utilities/test/utils.h"

namespace config::test {

YAML::Node createData() {
  YAML::Node data;
  data["a"]["b"]["c"] = 1;
  data["a"]["b"]["d"] = "test";
  data["a"]["b"]["e"] = std::vector<float>({1, 2, 3});
  data["a"]["b"]["f"] = std::map<std::string, int>({{"1_str", 1}, {"2_str", 2}});
  data["a"]["g"] = 3;
  return data;
}

TEST(YamlParsing, lookupNamespace) {
  YAML::Node data = createData();

  expectEqual(data, data);

  YAML::Node data_1 = YAML::Clone(data);
  data_1["a"]["b"]["c"] = 2;
  EXPECT_FALSE(internal::isEqual(data, data_1));

  YAML::Node data_2 = internal::lookupNamespace(data, "");
  // NOTE(lschmid): lookupNamespace returns a pointer, so this should be identity.
  EXPECT_TRUE(data == data_2);
  expectEqual(data, data_2);

  YAML::Node b = internal::lookupNamespace(data, "a/b");
  // NOTE(lschmid): lookupNamespace returns a pointer, so this should be identity.
  EXPECT_TRUE(b == data["a"]["b"]);
  expectEqual(b, data["a"]["b"]);

  YAML::Node b2 = internal::lookupNamespace(YAML::Clone(data), "a/b");

  expectEqual(b2, data["a"]["b"]);

  YAML::Node c = internal::lookupNamespace(data, "a/b/c");
  EXPECT_TRUE(c.IsScalar());
  EXPECT_EQ(c.as<int>(), 1);

  YAML::Node invalid = internal::lookupNamespace(data, "a/b/c/d");
  EXPECT_FALSE(invalid.IsDefined());
  EXPECT_FALSE(static_cast<bool>(invalid));

  // Make sure the input node is not modified.
  expectEqual(data, createData());
}

TEST(YamlParsing, moveDownNamespace) {
  YAML::Node data = createData();

  internal::moveDownNamespace(data, "");
  expectEqual(data, createData());

  YAML::Node expected_data;
  expected_data["a"]["b"]["c"] = createData();
  internal::moveDownNamespace(data, "a/b/c");
  expectEqual(data, expected_data);
}

TEST(YamlParsing, parsefromYaml) {
  DefaultConfig config;
  YAML::Node data = DefaultConfig::modifiedValues();
  std::string error;

  internal::YamlParser::fromYaml(data, "i", config.i, "", error);
  EXPECT_EQ(config.i, 2);

  internal::YamlParser::fromYaml(data, "f", config.f, "", error);
  EXPECT_EQ(config.f, -1.f);

  internal::YamlParser::fromYaml(data, "d", config.d, "", error);
  EXPECT_EQ(config.d, 3.1415926);

  internal::YamlParser::fromYaml(data, "b", config.b, "", error);
  EXPECT_EQ(config.b, false);

  internal::YamlParser::fromYaml(data, "u8", config.u8, "", error);
  EXPECT_EQ(config.u8, 255);

  internal::YamlParser::fromYaml(data, "s", config.s, "", error);
  EXPECT_EQ(config.s, "a different test string");

  internal::YamlParser::fromYaml(data, "vec", config.vec, "", error);
  EXPECT_EQ(config.vec, std::vector<int>({2, 3, 4, 5}));

  internal::YamlParser::fromYaml(data, "map", config.map, "", error);
  const std::map<std::string, int> map({{"x", 24}, {"y", 25}, {"z", 26}});
  EXPECT_EQ(config.map, map);

  internal::YamlParser::fromYaml(data, "set", config.set, "", error);
  EXPECT_EQ(config.set, std::set<float>({11.11, 22.22, 33.33, 44.44}));

  internal::YamlParser::fromYaml(data, "mat", config.mat, "", error);
  Eigen::Matrix3d mat;
  mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  EXPECT_EQ(config.mat, mat);
}

TEST(YamlParsing, conversionFailure) {
  DefaultConfig config;
  YAML::Node node = YAML::Load(R"yaml(
vec: "Invalid Value to create a vector from"
map: "Invalid Value to create a map from"
mat:
  - [1, 2, 3]
  - [4, 5, 6]
  - [7, 8, 9, 10]
i: "Value that can't be cast to int"
my_enum: "D"
sub_ns:
  i: "Value that can't be cast to int"
  nested_ns:
    i: "Value that can't be cast to int"
  )yaml");
  internal::MetaData data = internal::Visitor::setValues(config, node);
  std::vector<std::unique_ptr<internal::Warning>> errors;
  data.performOnAll([&errors](const internal::MetaData& d) {
    for (const auto& error : d.errors) {
      errors.emplace_back(error->clone());
    }
  });
  EXPECT_EQ(data.errors.size(), 5ul);
  ASSERT_EQ(errors.size(), 7ul);
  EXPECT_EQ(errors[0]->name(), "i");
  EXPECT_EQ(errors[1]->name(), "vec");
  EXPECT_EQ(errors[2]->name(), "map");
  EXPECT_EQ(errors[3]->name(), "mat");
  EXPECT_EQ(errors[4]->name(), "my_enum");
  EXPECT_EQ(errors[5]->name(), "i");
  EXPECT_EQ(errors[6]->name(), "i");
  EXPECT_EQ(errors[1]->message(), "Data is not a sequence");
  EXPECT_EQ(errors[2]->message(), "Data is not a map");
  EXPECT_EQ(errors[4]->message(), "Name 'D' is out of bounds for enum with names ['A', 'B', 'C']");
}

TEST(YamlParsing, setValues) {
  YAML::Node data = DefaultConfig::modifiedValues();
  DefaultConfig config;
  internal::MetaData meta_data = internal::Visitor::setValues(config, data);
  config.expectModifiedValues();
  EXPECT_FALSE(meta_data.hasErrors());

  // Make sure the input node is not modified.
  expectEqual(data, DefaultConfig::modifiedValues());

  // Test that the config is not modified if no data is found is empty.
  data = YAML::Node();
  meta_data = internal::Visitor::setValues(config, data);
  config.expectModifiedValues();
  EXPECT_FALSE(meta_data.hasErrors());

  data["i"] = "Not an int";
  data["u8"] = -1;
  data["d"] = static_cast<long double>(std::numeric_limits<double>::max()) * 2;
  data["vec"] = std::map<std::string, int>({{"1_str", 1}, {"2_str", 2}});
  data["map"] = std::vector<int>({1, 2, 3});
  data["set"] = std::vector<float>({11.11, 22.22, 33.33, 44.44, 11.11});
  data["mat"].push_back(std::vector<int>({1, 2, 3}));
  data["mat"].push_back(std::vector<int>({1, 2, 3, 4}));
  data["mat"].push_back(std::vector<int>({1, 2, 3}));
  data["my_enum"] = "OutOfList";
  meta_data = internal::Visitor::setValues(config, data);
  config.expectModifiedValues();
  EXPECT_TRUE(meta_data.hasErrors());
  EXPECT_EQ(meta_data.errors.size(), 8ul);
}

TEST(YamlParsing, getValues) {
  DefaultConfig config;
  internal::MetaData meta_data = internal::Visitor::getValues(config);

  config.expectDefaultValues();
  expectEqual(meta_data.data, DefaultConfig::defaultValues());
  EXPECT_FALSE(meta_data.hasErrors());
  EXPECT_EQ(meta_data.errors.size(), 0ul);
  meta_data.performOnAll([](const internal::MetaData& d) {
    for (const auto& field : d.field_infos) {
      EXPECT_TRUE(field.is_default);
    }
  });
  EXPECT_EQ(meta_data.name, "DefaultConfig");

  internal::Visitor::setValues(config, DefaultConfig::modifiedValues());
  meta_data = internal::Visitor::getValues(config);
  config.expectModifiedValues();
  expectEqual(meta_data.data, DefaultConfig::modifiedValues());
  EXPECT_FALSE(meta_data.hasErrors());
  EXPECT_EQ(meta_data.errors.size(), 0ul);
  meta_data.performOnAll([](const internal::MetaData& d) {
    for (const auto& field : d.field_infos) {
      EXPECT_FALSE(field.is_default);
    }
  });
}

TEST(YamlParsing, configFromYaml) {
  const YAML::Node data = DefaultConfig::modifiedValues();
  auto config = fromYaml<DefaultConfig>(data);
  config.expectModifiedValues();

  // Make sure the input node is not modified.
  expectEqual(data, DefaultConfig::modifiedValues());
}

TEST(YamlParsing, configToYAML) {
  DefaultConfig config;
  expectEqual(toYaml(config), DefaultConfig::defaultValues());

  config = fromYaml<DefaultConfig>(DefaultConfig::modifiedValues());
  expectEqual(toYaml(config), DefaultConfig::modifiedValues());
}

}  // namespace config::test
