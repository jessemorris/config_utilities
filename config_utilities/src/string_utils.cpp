#include "config_utilities/internal/string_utils.h"

#include <algorithm>
#include <sstream>

namespace config::internal {

std::string printCenter(const std::string& text, int width, char symbol) {
  int first = std::max((width - static_cast<int>(text.length()) - 2) / 2, 0);
  std::string result = std::string(first, symbol) + " " + text + " ";
  result += std::string(std::max(width - static_cast<int>(result.length()), 0), symbol);
  return result;
}

std::vector<std::string> splitNamespace(const std::string& text, const std::string& delimiter) {
  std::vector<std::string> result;
  if (text.empty()) {
    return result;
  }
  std::string s = text;
  size_t pos = 0;
  while ((pos = s.find(delimiter)) != std::string::npos) {
    if (pos != 0) {
      result.push_back(s.substr(0, pos));
    }
    s.erase(0, pos + delimiter.length());
  }
  if (!s.empty()) {
    result.push_back(s);
  }
  return result;
}

std::string joinNamespace(const std::vector<std::string>& namespaces, const std::string& delimiter) {
  std::string result;
  if (namespaces.empty()) {
    return result;
  }
  result = namespaces[0];
  for (size_t i = 1; i < namespaces.size(); ++i) {
    result += delimiter + namespaces[i];
  }
  return result;
}

std::string joinNamespace(const std::string& namespace_1,
                          const std::string& namespace_2,
                          const std::string& delimiter) {
  // Ensure the final formatting is uniform
  std::vector<std::string> ns_1 = splitNamespace(namespace_1, delimiter);
  const std::vector<std::string> ns_2 = splitNamespace(namespace_2, delimiter);
  ns_1.insert(ns_1.end(), ns_2.begin(), ns_2.end());
  return joinNamespace(ns_1, delimiter);
}

std::string dataToString(const YAML::Node& data) {
  switch (data.Type()) {
    case YAML::NodeType::Scalar: {
      // NOTE(lschmid): All YAML scalars should implement the ostream<< operator.
      std::stringstream ss;
      ss << data;
      return ss.str();
    }
    case YAML::NodeType::Sequence: {
      std::string result = "[";
      for (size_t i = 0; i < data.size(); ++i) {
        result += dataToString(data[i]);
        if (i < data.size() - 1) {
          result += ", ";
        }
      }
      result += "]";
      return result;
    }
    case YAML::NodeType::Map: {
      std::string result = "{";
      bool has_data = false;
      for (const auto& kv_pair : data) {
        has_data = true;
        result += dataToString(kv_pair.first) + ": " + dataToString(kv_pair.second) + ", ";
      }
      if (has_data) {
        result = result.substr(0, result.length() - 2);
      }
      result += "}";
      return result;
    }
    default:
      return kInvalidField;
  }
}

std::vector<size_t> findAllSubstrings(const std::string& text, const std::string& substring) {
  std::vector<size_t> result;
  size_t pos = text.find(substring, 0);
  while (pos != std::string::npos) {
    result.push_back(pos);
    pos = text.find(substring, pos + 1);
  }
  return result;
}

std::string pruneTrailingWhitespace(const std::string& text) {
  size_t i;
  for (i = 0; i < text.size(); ++i) {
    if (text.at(text.size() - i - 1) != ' ') {
      break;
    }
  }

  if (!i) {
    return text;
  }

  return text.substr(0, text.size() - i);
}

std::string pruneLeadingWhitespace(const std::string& text) {
  const size_t pos = text.find_first_not_of(' ');
  if (pos == std::string::npos) {
    return "";
  }
  return text.substr(pos);
}

std::string pruneWhitespace(const std::string& text) { return pruneLeadingWhitespace(pruneTrailingWhitespace(text)); }

std::string wrapString(const std::string& str, size_t width, size_t indent, bool indent_first_line) {
  std::string result;
  std::string remaining = str;
  const size_t length = width - indent;

  // Format the first line indent.
  if (indent_first_line) {
    result = std::string(indent, ' ');
  } else {
    const size_t first_line_length = std::min(indent, str.length());
    result = str.substr(0, first_line_length);
    remaining = str.substr(first_line_length);
  }

  if (remaining.empty()) {
    return pruneTrailingWhitespace(result);
  }

  // Wrap all other lines within the indent range.
  bool is_first_line = true;
  while (!remaining.empty()) {
    std::string next_line = remaining.substr(0, std::min(length, remaining.length()));
    remaining = remaining.substr(next_line.size());

    // Prune leading and trailing whitespace of all lines except the first.
    if (!is_first_line) {
      while (!next_line.empty() && next_line.at(0) == ' ') {
        next_line = next_line.substr(1);
        if (!remaining.empty()) {
          next_line += remaining.at(0);
          remaining = remaining.substr(1);
        }
      }
    }
    next_line = pruneTrailingWhitespace(next_line);
    if (next_line.empty()) {
      return result;
    }

    // Aggregate line.
    if (!is_first_line) {
      result += "\n" + std::string(indent, ' ');
    }
    is_first_line = false;
    result += next_line;
  }

  return result;
}

}  // namespace config::internal
