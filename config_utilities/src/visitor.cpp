

#include "config_utilities/internal/visitor.h"

#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/settings.h"

namespace config::internal {

thread_local std::vector<Visitor*> Visitor::instances = {};

Visitor::~Visitor() { instances.pop_back(); }

bool Visitor::hasInstance() { return !instances.empty(); }

Visitor::Visitor(Mode _mode, std::string _name_space, std::string _name_prefix)
    : mode(_mode), name_space(std::move(_name_space)), name_prefix(std::move(_name_prefix)) {
  // Create instances in a stack per thread and store the reference to it.
  instances.emplace_back(this);
}

Visitor& Visitor::instance() {
  if (instances.empty()) {
    // This should never happen as visitors are managed internally. Caught here for debugging.
    throw std::runtime_error("Visitor instance was accessed but no visitor was created before.");
  }
  return *instances.back();
}

void Visitor::extractErrors() {
  // Move the errors from the parser and checker to the meta data.
  data.errors.insert(data.errors.end(), parser.getErrors().begin(), parser.getErrors().end());
  data.errors.insert(data.errors.end(), checker.getWarnings().begin(), checker.getWarnings().end());
  checker.resetWarnings();
  parser.resetErrors();
}

void Visitor::visitName(const std::string& name) { Visitor::instance().data.name = name; }

void Visitor::visitCheckCondition(bool condition, const std::string& error_message) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode != Visitor::Mode::kCheck) {
    return;
  }
  visitor.checker.setFieldNamePrefix(visitor.name_prefix);
  visitor.checker.checkCondition(condition, error_message);
}

std::optional<YAML::Node> Visitor::visitVirtualConfig(bool is_set, bool is_optional, const std::string& type) {
  Visitor& visitor = Visitor::instance();
  visitor.data.is_virtual_config = true;

  if (visitor.mode == Visitor::Mode::kCheck) {
    if (!is_set && !is_optional) {
      // The config is required and not set.
      visitor.checker.checkCondition(
          false, "Variable config '" + visitor.data.current_field_name + "' is required but not set.");
    }
  }

  if (visitor.mode == Visitor::Mode::kGet) {
    if (is_set) {
      // Also write the type param back to file.
      visitor.parser.toYaml(
          Settings::instance().factory_type_param_name, type, visitor.name_space, visitor.name_prefix);
    }
  }

  if (visitor.mode == Visitor::Mode::kSet) {
    // Return the data to intialize the virtual config if this is the first time setting it.
    return lookupNamespace(visitor.parser.getNode(), visitor.name_space);
  }

  return std::nullopt;
}

}  // namespace config::internal
