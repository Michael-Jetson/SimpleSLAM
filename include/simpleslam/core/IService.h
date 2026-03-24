#pragma once

namespace SimpleSLAM {

/// Minimal base for all components registered in SystemContext.
/// Provides a virtual destructor so dynamic_pointer_cast works.
class IService {
public:
    virtual ~IService() = default;
};

} // namespace SimpleSLAM
