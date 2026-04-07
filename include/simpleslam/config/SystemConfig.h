#pragma once

#include <string>

#include <simpleslam/config/PipelineConfig.h>
#include <simpleslam/infrastructure/LogService.h>

namespace SimpleSLAM {

struct SystemConfig {
    struct Runtime {
        bool offline_mode = true;
        bool use_dataset_clock = false;
        double dataset_clock_speed = 1.0;
    };

    PipelineConfig pipeline;
    LogConfig logging;
    Runtime runtime;
};

} // namespace SimpleSLAM
