#include <simpleslam/config/ConfigLoader.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace SimpleSLAM {
namespace {

using ProcessNoiseMatrix = Eigen::Matrix<double, 18, 18>;

std::string toUpperCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    return value;
}

bool isIntegralScalar(const std::string& scalar) {
    return scalar.find('.') == std::string::npos &&
           scalar.find('e') == std::string::npos &&
           scalar.find('E') == std::string::npos;
}

ProcessNoiseMatrix loadProcessNoiseDiag(const YAML::Node& node) {
    if (!node.IsSequence() || node.size() != 18U) {
        throw std::runtime_error("iekf.process_noise_diag must contain 18 elements");
    }

    ProcessNoiseMatrix matrix = ProcessNoiseMatrix::Zero();
    for (int i = 0; i < 18; ++i) {
        matrix(i, i) = node[i].as<double>();
    }
    return matrix;
}

std::vector<double> parseCsvValues(const std::filesystem::path& csv_path) {
    std::ifstream input(csv_path);
    if (!input) {
        throw std::runtime_error("Failed to open process noise file: " + csv_path.string());
    }

    std::vector<double> values;
    std::string line;
    while (std::getline(input, line)) {
        for (char& ch : line) {
            if (ch == ',' || ch == ';' || ch == '\t') {
                ch = ' ';
            }
        }

        std::istringstream line_stream(line);
        double value = 0.0;
        while (line_stream >> value) {
            values.push_back(value);
        }
    }

    return values;
}

ProcessNoiseMatrix loadProcessNoiseFile(const std::filesystem::path& yaml_path,
                                        const std::string& relative_or_absolute_path) {
    std::filesystem::path csv_path(relative_or_absolute_path);
    if (csv_path.is_relative()) {
        csv_path = yaml_path.parent_path() / csv_path;
    }

    const std::vector<double> values = parseCsvValues(csv_path);
    if (values.size() != 18U * 18U) {
        throw std::runtime_error("iekf.process_noise_file must contain 324 numeric values");
    }

    ProcessNoiseMatrix matrix;
    for (int i = 0; i < 18; ++i) {
        for (int j = 0; j < 18; ++j) {
            matrix(i, j) = values[static_cast<std::size_t>(i * 18 + j)];
        }
    }
    return matrix;
}

ProcessNoiseMatrix loadProcessNoise(const YAML::Node& iekf_node, const std::filesystem::path& yaml_path) {
    if (!iekf_node || !iekf_node.IsMap()) {
        return ProcessNoiseMatrix::Identity() * 0.01;
    }

    if (iekf_node["process_noise_file"]) {
        return loadProcessNoiseFile(yaml_path, iekf_node["process_noise_file"].as<std::string>());
    }
    if (iekf_node["process_noise_diag"]) {
        return loadProcessNoiseDiag(iekf_node["process_noise_diag"]);
    }
    if (iekf_node["process_noise"]) {
        return iekf_node["process_noise"].as<ProcessNoiseMatrix>();
    }

    return ProcessNoiseMatrix::Identity() * 0.01;
}

Node parseValueNode(const YAML::Node& node) {
    if (!node || node.IsNull()) {
        return Node();
    }

    if (node.IsScalar()) {
        return parseScalar(node);
    }

    if (node.IsSequence()) {
        Node::Array values;
        values.reserve(node.size());
        for (const auto& item : node) {
            values.push_back(parseValueNode(item));
        }
        return Node(std::move(values));
    }

    if (node.IsMap()) {
        return Node(loadParamsFromYAML(node));
    }

    throw std::runtime_error("Unsupported YAML node type in parameter tree");
}

} // namespace

namespace {

PipelineConfig loadPipelineConfigFromYaml(const YAML::Node& yaml, const std::string& yaml_path) {
    PipelineConfig config;

    const YAML::Node pipeline_node = yaml["pipeline"] ? yaml["pipeline"] : yaml;
    const YAML::Node sensors_node = yaml["sensors"] ? yaml["sensors"] : pipeline_node["sensors"];
    const YAML::Node iekf_node = yaml["iekf"] ? yaml["iekf"] : pipeline_node["iekf"];
    const YAML::Node keyframe_node = yaml["keyframe"] ? yaml["keyframe"] : pipeline_node["keyframe"];

    if (pipeline_node["type"]) {
        config.type = pipeline_node["type"].as<std::string>();
    }

    if (sensors_node && sensors_node.IsSequence()) {
        config.sensors.clear();
        for (const auto& s : sensors_node) {
            PipelineConfig::Sensor sensor;
            if (s["type"]) sensor.type = s["type"].as<std::string>();
            if (s["frequency"]) sensor.frequency = s["frequency"].as<double>();
            if (s["extrinsics"] && s["extrinsics"].IsSequence() && s["extrinsics"].size() == 16) {
                for (int i = 0; i < 16; ++i) {
                    sensor.extrinsics(i / 4, i % 4) = s["extrinsics"][i].as<double>();
                }
            }
            config.sensors.push_back(std::move(sensor));
        }
    }

    if (iekf_node && iekf_node.IsMap()) {
        if (iekf_node["max_iterations"])
            config.iekf.max_iterations = iekf_node["max_iterations"].as<int>();
        if (iekf_node["convergence_threshold"])
            config.iekf.convergence_threshold = iekf_node["convergence_threshold"].as<double>();
        config.iekf.process_noise = loadProcessNoise(iekf_node, std::filesystem::path(yaml_path));
    }

    if (keyframe_node && keyframe_node.IsMap()) {
        if (keyframe_node["translation_threshold"])
            config.keyframe.translation_threshold = keyframe_node["translation_threshold"].as<double>();
        if (keyframe_node["rotation_threshold"])
            config.keyframe.rotation_threshold = keyframe_node["rotation_threshold"].as<double>();
        if (keyframe_node["min_frame_interval"])
            config.keyframe.min_frame_interval = keyframe_node["min_frame_interval"].as<int>();
    }

    return config;
}

LogConfig loadLogConfigFromYaml(const YAML::Node& yaml) {
    LogConfig config;
    const YAML::Node infrastructure_node = yaml["infrastructure"] ? yaml["infrastructure"] : YAML::Node();
    const YAML::Node logging_node = yaml["logging"]
        ? yaml["logging"]
        : ((infrastructure_node && infrastructure_node.IsMap()) ? infrastructure_node["logging"] : YAML::Node());
    if (!logging_node || !logging_node.IsMap()) {
        return config;
    }

    if (logging_node["enable_console"]) config.enable_console = logging_node["enable_console"].as<bool>();
    if (logging_node["enable_file"]) config.enable_file = logging_node["enable_file"].as<bool>();
    if (logging_node["file_path"]) config.file_path = logging_node["file_path"].as<std::string>();
    if (logging_node["async"]) config.async = logging_node["async"].as<bool>();
    if (logging_node["async_queue_size"]) config.async_queue_size = logging_node["async_queue_size"].as<size_t>();
    if (logging_node["async_threads"]) config.async_threads = logging_node["async_threads"].as<size_t>();
    if (logging_node["console_pattern"]) config.console_pattern = logging_node["console_pattern"].as<std::string>();
    if (logging_node["file_pattern"]) config.file_pattern = logging_node["file_pattern"].as<std::string>();
    if (logging_node["tag_delim"]) config.tag_delim = logging_node["tag_delim"].as<std::string>();

    return config;
}

SystemConfig::Runtime loadRuntimeConfigFromYaml(const YAML::Node& yaml) {
    SystemConfig::Runtime config;
    if (yaml["mode"]) {
        const std::string mode = toUpperCopy(yaml["mode"].as<std::string>());
        config.offline_mode = (mode != "ONLINE");
    }

    const YAML::Node runtime_node = yaml["runtime"] ? yaml["runtime"] : YAML::Node();
    if (runtime_node && runtime_node.IsMap()) {
        if (runtime_node["offline_mode"]) config.offline_mode = runtime_node["offline_mode"].as<bool>();
        if (runtime_node["use_dataset_clock"]) config.use_dataset_clock = runtime_node["use_dataset_clock"].as<bool>();
        if (runtime_node["dataset_clock_speed"]) config.dataset_clock_speed = runtime_node["dataset_clock_speed"].as<double>();
    }

    return config;
}

} // namespace

PipelineConfig ConfigLoader::load(const std::string& yaml_path) {
    return loadSystem(yaml_path).pipeline;
}

SystemConfig ConfigLoader::loadSystem(const std::string& yaml_path) {
    const YAML::Node yaml = YAML::LoadFile(yaml_path);

    SystemConfig config;
    config.pipeline = loadPipelineConfigFromYaml(yaml, yaml_path);
    config.logging = loadLogConfigFromYaml(yaml);
    config.runtime = loadRuntimeConfigFromYaml(yaml);

    validate(config.pipeline);
    return config;
}

// Compatibility path: legacy callers that only care about pipeline-level config
// continue to use ConfigLoader::load(). New system assembly should prefer loadSystem().

void ConfigLoader::validate(const PipelineConfig& config) {
    if (config.sensors.empty()) {
        throw std::runtime_error("At least one sensor must be configured");
    }

    if (config.iekf.max_iterations <= 0) {
        throw std::runtime_error("iekf.max_iterations must be positive");
    }

    if (toUpperCopy(config.type) == "LIO") {
        const bool has_imu = std::any_of(config.sensors.begin(), config.sensors.end(), [](const auto& sensor) {
            return toUpperCopy(sensor.type) == "IMU";
        });
        if (!has_imu) {
            throw std::runtime_error("LIO pipeline requires at least one IMU sensor");
        }
    }
}

Node parseScalar(const YAML::Node& node) {
    const std::string tag = node.Tag();
    const std::string scalar = node.Scalar();

    if (tag == "!!str") {
        return Node(node.as<std::string>());
    }
    if (tag == "!!bool") {
        return Node(node.as<bool>());
    }
    if (tag == "!!int") {
        return Node(node.as<int>());
    }
    if (tag == "!!float") {
        return Node(node.as<double>());
    }

    if (scalar == "true" || scalar == "false") {
        return Node(node.as<bool>());
    }

    if (isIntegralScalar(scalar)) {
        try {
            return Node(node.as<int>());
        } catch (const YAML::BadConversion&) {
        }
    }

    try {
        return Node(node.as<double>());
    } catch (const YAML::BadConversion&) {
    }

    return Node(node.as<std::string>());
}

Params loadParamsFromYAML(const YAML::Node& yaml) {
    Params params;
    if (!yaml || yaml.IsNull()) {
        return params;
    }
    if (!yaml.IsMap()) {
        throw std::runtime_error("loadParamsFromYAML expects a YAML map");
    }

    for (const auto& kv : yaml) {
        const std::string key = kv.first.as<std::string>();
        params[key] = parseValueNode(kv.second);
    }

    return params;
}

} // namespace SimpleSLAM
