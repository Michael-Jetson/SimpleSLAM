#pragma once

#include "core/Registry.hpp"

namespace SimpleSLAM {

// Thin alias layer for module implementations.
class IModule : public ModuleBase {
public:
  using ModuleBase::ModuleBase;
  ~IModule() override = default;
};

} // namespace SimpleSLAM
