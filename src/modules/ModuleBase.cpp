#include <simpleslam/modules/ModuleBase.h>
#include <simpleslam/runtime/SystemContext.h>
#include <stdexcept>

namespace SimpleSLAM {

void ModuleBase::setContext(std::shared_ptr<SystemContext> ctx) {
    if (!ctx) {
        throw std::invalid_argument("ModuleBase::setContext requires non-null SystemContext");
    }
    ctx_ = std::move(ctx);
    log_ = ctx_->getLogger(name_);
}

} // namespace SimpleSLAM
