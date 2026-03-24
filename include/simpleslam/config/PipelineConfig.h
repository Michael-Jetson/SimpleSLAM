#pragma once

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

namespace SimpleSLAM {

struct PipelineConfig {
    struct Sensor {
        std::string type = "LiDAR";
        double frequency = 10.0;
        Eigen::Matrix4d extrinsics = Eigen::Matrix4d::Identity();
    };

    struct IEKF {
        int max_iterations = 5;
        double convergence_threshold = 1e-4;
        Eigen::Matrix<double, 18, 18> process_noise =
            Eigen::Matrix<double, 18, 18>::Identity() * 0.01;
    };

    struct KeyframeSelector {
        double translation_threshold = 0.5;
        double rotation_threshold = 10.0;
        int min_frame_interval = 5;
    };

    std::string type = "LIO";
    std::vector<Sensor> sensors;
    IEKF iekf;
    KeyframeSelector keyframe;
};

} // namespace SimpleSLAM

namespace YAML {
namespace simpleslam_config_detail {

template <int Rows, int Cols>
inline Node encodeFlatMatrix(const Eigen::Matrix<double, Rows, Cols>& mat) {
    Node node;
    for (int i = 0; i < Rows; ++i) {
        for (int j = 0; j < Cols; ++j) {
            node.push_back(mat(i, j));
        }
    }
    return node;
}

template <int Rows, int Cols>
inline bool decodeMatrixSequence(const Node& node, Eigen::Matrix<double, Rows, Cols>& mat) {
    if (!node.IsSequence()) {
        return false;
    }

    if (node.size() == static_cast<std::size_t>(Rows * Cols)) {
        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                mat(i, j) = node[i * Cols + j].as<double>();
            }
        }
        return true;
    }

    if (node.size() == static_cast<std::size_t>(Rows)) {
        for (int i = 0; i < Rows; ++i) {
            const auto row = node[i];
            if (!row.IsSequence() || row.size() != static_cast<std::size_t>(Cols)) {
                return false;
            }
            for (int j = 0; j < Cols; ++j) {
                mat(i, j) = row[j].as<double>();
            }
        }
        return true;
    }

    return false;
}

inline bool decodeProcessNoiseDiag(const Node& node, Eigen::Matrix<double, 18, 18>& mat) {
    if (!node.IsSequence() || node.size() != 18U) {
        return false;
    }

    mat.setZero();
    for (int i = 0; i < 18; ++i) {
        mat(i, i) = node[i].as<double>();
    }
    return true;
}

} // namespace simpleslam_config_detail

template <>
struct convert<Eigen::Matrix4d> {
    static Node encode(const Eigen::Matrix4d& mat) {
        return simpleslam_config_detail::encodeFlatMatrix<4, 4>(mat);
    }

    static bool decode(const Node& node, Eigen::Matrix4d& mat) {
        return simpleslam_config_detail::decodeMatrixSequence<4, 4>(node, mat);
    }
};

template <>
struct convert<Eigen::Matrix<double, 18, 18>> {
    static Node encode(const Eigen::Matrix<double, 18, 18>& mat) {
        return simpleslam_config_detail::encodeFlatMatrix<18, 18>(mat);
    }

    static bool decode(const Node& node, Eigen::Matrix<double, 18, 18>& mat) {
        return simpleslam_config_detail::decodeMatrixSequence<18, 18>(node, mat);
    }
};

template <>
struct convert<SimpleSLAM::PipelineConfig::Sensor> {
    static Node encode(const SimpleSLAM::PipelineConfig::Sensor& sensor) {
        Node node(NodeType::Map);
        node["type"] = sensor.type;
        node["frequency"] = sensor.frequency;
        node["extrinsics"] = sensor.extrinsics;
        return node;
    }

    static bool decode(const Node& node, SimpleSLAM::PipelineConfig::Sensor& sensor) {
        if (!node.IsMap()) {
            return false;
        }

        SimpleSLAM::PipelineConfig::Sensor parsed;
        if (node["type"]) {
            parsed.type = node["type"].as<std::string>();
        }
        if (node["frequency"]) {
            parsed.frequency = node["frequency"].as<double>();
        }
        if (node["extrinsics"]) {
            parsed.extrinsics = node["extrinsics"].as<Eigen::Matrix4d>();
        }

        sensor = std::move(parsed);
        return true;
    }
};

template <>
struct convert<SimpleSLAM::PipelineConfig::IEKF> {
    static Node encode(const SimpleSLAM::PipelineConfig::IEKF& iekf) {
        Node node(NodeType::Map);
        node["max_iterations"] = iekf.max_iterations;
        node["convergence_threshold"] = iekf.convergence_threshold;
        node["process_noise"] = iekf.process_noise;
        return node;
    }

    static bool decode(const Node& node, SimpleSLAM::PipelineConfig::IEKF& iekf) {
        if (!node.IsMap()) {
            return false;
        }

        SimpleSLAM::PipelineConfig::IEKF parsed;
        if (node["max_iterations"]) {
            parsed.max_iterations = node["max_iterations"].as<int>();
        }
        if (node["convergence_threshold"]) {
            parsed.convergence_threshold = node["convergence_threshold"].as<double>();
        }
        if (node["process_noise"]) {
            parsed.process_noise = node["process_noise"].as<Eigen::Matrix<double, 18, 18>>();
        } else if (node["process_noise_diag"]) {
            if (!simpleslam_config_detail::decodeProcessNoiseDiag(node["process_noise_diag"],
                                                                  parsed.process_noise)) {
                return false;
            }
        }

        iekf = std::move(parsed);
        return true;
    }
};

template <>
struct convert<SimpleSLAM::PipelineConfig::KeyframeSelector> {
    static Node encode(const SimpleSLAM::PipelineConfig::KeyframeSelector& keyframe) {
        Node node(NodeType::Map);
        node["translation_threshold"] = keyframe.translation_threshold;
        node["rotation_threshold"] = keyframe.rotation_threshold;
        node["min_frame_interval"] = keyframe.min_frame_interval;
        return node;
    }

    static bool decode(const Node& node, SimpleSLAM::PipelineConfig::KeyframeSelector& keyframe) {
        if (!node.IsMap()) {
            return false;
        }

        SimpleSLAM::PipelineConfig::KeyframeSelector parsed;
        if (node["translation_threshold"]) {
            parsed.translation_threshold = node["translation_threshold"].as<double>();
        }
        if (node["rotation_threshold"]) {
            parsed.rotation_threshold = node["rotation_threshold"].as<double>();
        }
        if (node["min_frame_interval"]) {
            parsed.min_frame_interval = node["min_frame_interval"].as<int>();
        }

        keyframe = std::move(parsed);
        return true;
    }
};

template <>
struct convert<SimpleSLAM::PipelineConfig> {
    static Node encode(const SimpleSLAM::PipelineConfig& config) {
        Node node(NodeType::Map);
        node["type"] = config.type;
        node["sensors"] = config.sensors;
        node["iekf"] = config.iekf;
        node["keyframe"] = config.keyframe;
        return node;
    }

    static bool decode(const Node& node, SimpleSLAM::PipelineConfig& config) {
        if (!node.IsMap()) {
            return false;
        }

        SimpleSLAM::PipelineConfig parsed;
        if (node["type"]) {
            parsed.type = node["type"].as<std::string>();
        }
        if (node["sensors"]) {
            parsed.sensors = node["sensors"].as<std::vector<SimpleSLAM::PipelineConfig::Sensor>>();
        }
        if (node["iekf"]) {
            parsed.iekf = node["iekf"].as<SimpleSLAM::PipelineConfig::IEKF>();
        }
        if (node["keyframe"]) {
            parsed.keyframe = node["keyframe"].as<SimpleSLAM::PipelineConfig::KeyframeSelector>();
        }

        config = std::move(parsed);
        return true;
    }
};

} // namespace YAML
