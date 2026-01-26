#pragma once

#include "core/Registry.hpp"

namespace SimpleSLAM {

// Thin alias layer for plugin implementations.
class IPlugin : public PluginBase {
public:
  using PluginBase::PluginBase;
  ~IPlugin() override = default;
};

} // namespace SimpleSLAM
