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

#include <glog/logging.h>

#include "rcl_interfaces/msg/parameter.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter.hpp"


#include "config_utilities/factory.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/parsing/yaml.h"  // NOTE(lschmid): This pulls in more than needed buyt avoids code duplication.

namespace config {

namespace internal {

inline YAML::Node rclParameterToYaml(const rclcpp::Parameter& parameter) {

    auto node_from_sequence = [](auto iterable) -> YAML::Node {
        YAML::Node node(YAML::NodeType::Sequence);
        for (const auto& e : iterable) {
            node.push_back(e);
        }
        return node;
    };


    //TODO: clean up
    //have to do this as std::vector<bool> doesnt actually contain a bool (its a bitfield!)
    auto node_from_bool_sequence = [](const std::vector<bool>& iterable) -> YAML::Node {
        YAML::Node node(YAML::NodeType::Sequence);
        //do implicit casting here
        for (const bool e : iterable) {
            node.push_back(e);
        }
        return node;
    };

    switch (parameter.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            return YAML::Node(parameter.as_bool());
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            return YAML::Node(static_cast<int>(parameter.as_int())); //as_int() -> int64_t
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            return YAML::Node(parameter.as_double());
        case rclcpp::ParameterType::PARAMETER_STRING:
            return YAML::Node(parameter.as_string());
        case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
            return node_from_sequence(parameter.as_byte_array());
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
            return node_from_bool_sequence(parameter.as_bool_array());
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
            return node_from_sequence(parameter.as_integer_array());
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            return node_from_sequence(parameter.as_double_array());
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            return node_from_sequence(parameter.as_string_array());
        default:
            //TODO: handle case of PARAMETER_NOT_SET?
            return YAML::Node();
    }
} 


inline YAML::Node rosToYaml(rclcpp::Node::SharedPtr node) {
    YAML::Node yaml_node;
  
  //from a few discussions (https://github.com/ros2/rclcpp/issues/196 and others) it appears 
  //that requesting a depth=0 is the same as asking for all depths
  auto parameter_list_result = node->list_parameters(std::vector<std::string>{}, 0u);

  std::cout << "node namespace " << node->get_namespace() << std::endl;
  std::cout << "node name " << node->get_name() << std::endl;
  std::cout << "node qualify " << node->get_fully_qualified_name() << std::endl;
  std::cout << "node effective " << node->get_effective_namespace() << std::endl;

    for(auto prefix : parameter_list_result.prefixes) std::cout << prefix << std::endl;
    std::cout << "names" << std::endl;
    for(auto name : parameter_list_result.names) std::cout << name << std::endl;

  //have to resolve name?
  const std::vector<rclcpp::Parameter> parameters = node->get_parameters (parameter_list_result.names);

    //TODO: not sure yet how to handle subnamespaces
    for(const rclcpp::Parameter& parameter : parameters) {
        yaml_node[parameter.get_name()] = rclParameterToYaml(parameter);
    }

  return yaml_node;
}

}  // namespace internal


/*
 * @brief Loads a config from a yaml node.
 *
 * @tparam ConfigT The config type. This can also be a VirtualConfig<BaseT> or a std::vector<ConfigT>.
 * @param nh The ROS nodehandle to create the config from.
 * @param name_space Optionally specify a name space to create the config from. Separate names with slashes '/'.
 * Example: "my_config/my_sub_config".
 * @returns The config.
 */

//TODO: not sure if subnamespacing will work exactly the same...
template <typename ConfigT>
ConfigT fromRos(rclcpp::Node::SharedPtr node, const std::string& sub_namespace = "") {
    rclcpp::Node::SharedPtr ns_node = node;
    if(!sub_namespace.empty()) ns_node = node->create_sub_node (sub_namespace);
  const YAML::Node yaml_node = internal::rosToYaml(ns_node);
  return fromYaml<ConfigT>(yaml_node, "");
}

// /**
//  * @brief Create a derived type object based on a the data stored in a yaml node. All derived types need to be
//  * registered to the factory using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct. They
//  * need to implement a config as a public member struct named 'Config' and use the config as the first constructor
//  * argument.
//  *
//  * @tparam BaseT Type of the base class to be constructed.
//  * @tparam Args Other constructor arguments. Note that each unique set of constructor arguments will result in a
//  * different base-entry in the factory.
//  * @param nh The ROS nodehandle containing the type identifier as a param and the data to create the config.
//  * @param args Other constructor arguments.
//  * @returns Unique pointer of type base that contains the derived object.
//  */
// template <typename BaseT, typename... ConstructorArguments>
// std::unique_ptr<BaseT> createFromROS(const ros::NodeHandle& nh, ConstructorArguments... args) {
//   return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(internal::rosToYaml(nh), args...);
// }

// /**
//  * @brief Create a derived type object based on a the data stored in a yaml node. All derived types need to be
//  * registered to the factory using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct. They
//  * need to implement a config as a public member struct named 'Config' and use the config as the first constructor
//  * argument.
//  *
//  * @tparam BaseT Type of the base class to be constructed.
//  * @tparam Args Other constructor arguments. Note that each unique set of constructor arguments will result in a
//  * different base-entry in the factory.
//  * @param nh The ROS nodehandle containing the type identifier as a param and the data to create the config.
//  * @param name_space Optionally specify a name space to create the object from. Separate names with
//  * slashes '/'. Example: "my_config/my_sub_config".
//  * @param args Other constructor arguments.
//  * @returns Unique pointer of type base that contains the derived object.
//  */
// template <typename BaseT, typename... ConstructorArguments>
// std::unique_ptr<BaseT> createFromROSWithNamespace(const ros::NodeHandle& nh,
//                                                   const std::string& name_space,
//                                                   ConstructorArguments... args) {
//   ros::NodeHandle ns_nh = ros::NodeHandle(nh, name_space);
//   return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(internal::rosToYaml(ns_nh), args...);
// }

}  // namespace config
