#pragma once

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <utility>

#include <simpleslam/runtime/Permission.h>

namespace SimpleSLAM {

namespace detail {

template<Permission P>
inline constexpr bool kCanRead =
    P == Permission::ReadOnly || P == Permission::ReadWrite;

template<Permission P>
inline constexpr bool kCanWrite = P == Permission::ReadWrite;

template<typename T>
inline void validateResourceAccess(const std::shared_ptr<T>& resource,
                                   const std::shared_mutex* mutex) {
    if (!resource) {
        throw std::runtime_error("ResourceHandle access on null resource");
    }
    if (mutex == nullptr) {
        throw std::runtime_error("ResourceHandle access on null mutex");
    }
}

} // namespace detail

template<typename T>
class ReadGuard {
public:
    ReadGuard(std::shared_ptr<T> resource, std::shared_mutex* mutex)
        : resource_(std::move(resource)) {
        detail::validateResourceAccess(resource_, mutex);
        lock_ = std::shared_lock<std::shared_mutex>(*mutex);
        ptr_ = resource_.get();
    }

    const T* operator->() const noexcept { return ptr_; }
    const T& operator*() const noexcept { return *ptr_; }

private:
    std::shared_ptr<T> resource_;
    std::shared_lock<std::shared_mutex> lock_;
    const T* ptr_ = nullptr;
};

template<typename T>
class WriteGuard {
public:
    WriteGuard(std::shared_ptr<T> resource, std::shared_mutex* mutex)
        : resource_(std::move(resource)) {
        detail::validateResourceAccess(resource_, mutex);
        lock_ = std::unique_lock<std::shared_mutex>(*mutex);
        ptr_ = resource_.get();
    }

    T* operator->() noexcept { return ptr_; }
    T& operator*() noexcept { return *ptr_; }
    const T* operator->() const noexcept { return ptr_; }
    const T& operator*() const noexcept { return *ptr_; }

private:
    std::shared_ptr<T> resource_;
    std::unique_lock<std::shared_mutex> lock_;
    T* ptr_ = nullptr;
};

template<typename T, Permission P>
class ResourceHandle {
public:
    ResourceHandle() = default;

    ResourceHandle(std::shared_ptr<T> resource, std::shared_mutex* mutex)
        : resource_(std::move(resource)), mutex_(mutex) {}

    [[nodiscard]] bool valid() const noexcept {
        return static_cast<bool>(resource_) && mutex_ != nullptr;
    }

    [[nodiscard]] auto read() const requires(detail::kCanRead<P>) {
        return ReadGuard<T>(resource_, mutex_);
    }

    [[nodiscard]] auto write() requires(detail::kCanWrite<P>) {
        return WriteGuard<T>(resource_, mutex_);
    }

private:
    std::shared_ptr<T> resource_;
    std::shared_mutex* mutex_ = nullptr;
};

} // namespace SimpleSLAM
