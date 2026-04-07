#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <simpleslam/config/Params.h>
#include <simpleslam/core/IService.h>
#include <simpleslam/infrastructure/LogService.h>

namespace SimpleSLAM {

class SystemContext;

class ModuleBase : public IService {
public:
    ModuleBase(std::string name, Params params)
        : name_(std::move(name)), params_(std::move(params)) {}

    ~ModuleBase() override = default;

    [[nodiscard]] const std::string& name() const noexcept { return name_; }
    [[nodiscard]] const Params& params() const noexcept { return params_; }

    void setContext(std::shared_ptr<SystemContext> ctx);

    [[nodiscard]] virtual std::vector<std::string> declaredDependencies() const {
        return {};
    }

    virtual bool Init() = 0;

protected:
    std::string name_;
    Params params_;
    std::shared_ptr<SystemContext> ctx_;
    Log log_;
};

} // namespace SimpleSLAM
