#pragma once

#include <string>

#include <yaml-cpp/yaml.h>

#include <simpleslam/config/Params.h>
#include <simpleslam/config/PipelineConfig.h>

namespace SimpleSLAM {

class ConfigLoader {
public:
    static PipelineConfig load(const std::string& yaml_path);
    static void validate(const PipelineConfig& config);
};

Node parseScalar(const YAML::Node& node);
Params loadParamsFromYAML(const YAML::Node& yaml);

} // namespace SimpleSLAM
