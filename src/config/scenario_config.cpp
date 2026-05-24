#include "ros2_fault_injection/config/scenario_config.hpp"

#include <chrono>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace ros2_fault_injection
{
namespace
{

std::string required_string(const YAML::Node & node, const std::string & key)
{
  if (!node[key]) {
    throw std::runtime_error("Missing required key: " + key);
  }

  return node[key].as<std::string>();
}

std::string node_to_string(const YAML::Node & node)
{
  if (node.IsScalar()) {
    return node.as<std::string>();
  }

  throw std::runtime_error("Expected scalar YAML value");
}

InjectorConfig parse_injector(const YAML::Node & node)
{
  InjectorConfig config;
  config.id = required_string(node, "id");
  config.type = required_string(node, "type");

  if (config.type == "trigger_service") {
    TriggerServiceEndpointConfig service;
    service.proxy_service = required_string(node, "proxy_service");
    service.target_service = required_string(node, "target_service");
    config.trigger_service = service;
    return config;
  }

  TopicEndpointConfig topic;
  topic.input_topic = required_string(node, "input_topic");
  topic.output_topic = required_string(node, "output_topic");

  if (node["qos_depth"]) {
    topic.qos_depth = node["qos_depth"].as<std::size_t>();
  }

  config.topic = topic;
  return config;
}

FaultConfig parse_fault(const YAML::Node & node)
{
  FaultConfig fault;
  fault.id = required_string(node, "id");
  fault.injector_id = required_string(node, "injector_id");

  if (node["start"]) {
    const auto seconds = node["start"].as<double>();
    fault.start = std::chrono::milliseconds{static_cast<int64_t>(seconds * 1000.0)};
  }

  if (node["duration"]) {
    const auto seconds = node["duration"].as<double>();
    fault.duration = std::chrono::milliseconds{static_cast<int64_t>(seconds * 1000.0)};
  }

  if (node["config"]) {
    // Store values as strings for now. Individual injectors decide how to
    // interpret keys such as x_bias, drop_probability, or delay_ms.
    for (const auto & item : node["config"]) {
      const auto key = item.first.as<std::string>();
      const auto value = node_to_string(item.second);
      fault.config[key] = value;
    }
  }

  return fault;
}

}  // namespace

ScenarioConfig load_scenario_config(const std::string & path)
{
  const auto root = YAML::LoadFile(path);

  ScenarioConfig scenario;

  if (root["injectors"]) {
    for (const auto & injector_node : root["injectors"]) {
      scenario.injectors.push_back(parse_injector(injector_node));
    }

    if (!scenario.injectors.empty()) {
      scenario.injector = scenario.injectors.front();
    }
  } else if (root["injector"]) {
    scenario.injector = parse_injector(root["injector"]);
    scenario.injectors.push_back(scenario.injector);
  } else {
    throw std::runtime_error("Scenario is missing required 'injector' or 'injectors' block");
  }

  if (root["faults"]) {
    for (const auto & fault_node : root["faults"]) {
      auto fault = parse_fault(fault_node);

      // active_on_startup: true means the fault is active as soon as the
      // framework starts. Keep active as a deprecated alias for older scenarios.
      const bool active_on_startup =
        (fault_node["active_on_startup"] && fault_node["active_on_startup"].as<bool>()) ||
        (fault_node["active"] && fault_node["active"].as<bool>());

      if (active_on_startup) {
        scenario.initially_active_faults.push_back(fault.id);
      }

      scenario.faults.push_back(std::move(fault));
    }
  }

  return scenario;
}

const InjectorConfig * find_injector(
  const ScenarioConfig & scenario,
  const std::string & injector_id)
{
  for (const auto & injector : scenario.injectors) {
    if (injector.id == injector_id) {
      return &injector;
    }
  }

  return nullptr;
}

}  // namespace ros2_fault_injection
