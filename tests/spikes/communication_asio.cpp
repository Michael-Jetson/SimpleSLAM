// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors
// Design spike: Asio execution, Latest delivery, async service and parameter commit.
// This is a standalone experiment, not a replacement for the production bus.

#include <utility>
#if defined(SIMPLESLAM_SPIKE_STANDALONE_ASIO)
#include <asio.hpp>
namespace net = asio;
using ErrorCode = std::error_code;
#else
#include <boost/asio.hpp>
namespace net = boost::asio;
using ErrorCode = boost::system::error_code;
#endif

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

using Strand = net::strand<net::io_context::executor_type>;

void check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

// A single-consumer Latest slot. One pending wake-up, payload overwritten in place.
// Subscription lifetime, named routing and general QoS are outside this spike.
class LatestSlot final : public std::enable_shared_from_this<LatestSlot> {
public:
    LatestSlot(Strand owner, std::function<void(int)> callback)
        : owner_(std::move(owner)), callback_(std::move(callback)) {}

    void publish(int value) {
        std::lock_guard lock(mutex_);
        latest_ = value;
        if (pending_) return;
        pending_ = true;
        try {
            net::post(owner_, [self = shared_from_this()] { self->deliver(); });
        } catch (...) {
            pending_ = false;
            throw;
        }
    }

    int latest() const {
        std::lock_guard lock(mutex_);
        return latest_;
    }

private:
    void deliver() {
        int value;
        {
            std::lock_guard lock(mutex_);
            value = latest_;
            pending_ = false;
        }
        check(owner_.running_in_this_thread(), "Latest escaped its owner strand");
        callback_(value);
    }

    Strand owner_;
    std::function<void(int)> callback_;
    mutable std::mutex mutex_;
    int latest_ = 0;
    bool pending_ = false;
};

// Result and timeout both commit on the client strand; at most one completion.
// The server executes on its own strand. Timeout does not undo server side effects.
template <class Response>
struct PendingCall final {
    net::steady_timer timer;
    bool completed = false;
    std::function<void(std::optional<Response>)> callback;

    PendingCall(Strand client, std::function<void(std::optional<Response>)> cb)
        : timer(std::move(client)), callback(std::move(cb)) {}

    void finish(std::optional<Response> response) {
        if (completed) return;
        completed = true;
        timer.cancel();
        callback(std::move(response));
    }
};

template <class Request, class Handler, class Completion>
void asyncCall(Strand client, Strand server, Request request,
               std::chrono::steady_clock::duration timeout,
               Handler handler, Completion completion) {
    using Response = std::invoke_result_t<Handler, Request>;
    auto pending = std::make_shared<PendingCall<Response>>(client, std::move(completion));
    net::post(client, [=]() mutable {
        pending->timer.expires_after(timeout);
        pending->timer.async_wait([pending](ErrorCode error) {
            if (!error) pending->finish(std::nullopt);
        });
        net::post(server, [=]() mutable {
            Response response = handler(std::move(request));
            net::post(client, [pending, response = std::move(response)]() mutable {
                pending->finish(std::move(response));
            });
        });
    });
}

void testModuleSerialization() {
    net::io_context io;
    auto work = net::make_work_guard(io);
    auto owner = net::make_strand(io);
    std::atomic<int> active{0};
    std::atomic<int> violations{0};
    int processed = 0;
    auto producer = [&] {
        for (int i = 0; i < 2000; ++i) {
            net::post(owner, [&] {
                if (active.fetch_add(1) != 0) ++violations;
                ++processed;
                std::this_thread::yield();
                active.fetch_sub(1);
            });
        }
    };
    std::jthread worker1([&] { io.run(); });
    std::jthread worker2([&] { io.run(); });
    std::jthread publisher1(producer);
    std::jthread publisher2(producer);
    publisher1.join();
    publisher2.join();
    work.reset();
    worker1.join();
    worker2.join();
    check(processed == 4000 && violations == 0, "Module handlers overlapped or were lost");
}

void testLatestAndReentrancy() {
    net::io_context io;
    auto owner = net::make_strand(io);
    std::vector<int> received;
    std::shared_ptr<LatestSlot> slot;
    slot = std::make_shared<LatestSlot>(owner, [&](int value) {
        received.push_back(value);
        check(slot->latest() >= value, "Cannot query Latest from its callback");
        if (value == 3) slot->publish(4);
    });
    slot->publish(1);
    slot->publish(2);
    slot->publish(3);
    check(received.empty(), "post executed the callback inline");
    io.poll();
    check(received == std::vector<int>({3, 4}), "Latest coalescing or reentrant publish failed");
}

void testService(bool force_timeout) {
    net::io_context client_io;
    net::io_context server_io;
    auto client = net::make_strand(client_io);
    auto server = net::make_strand(server_io);
    int completions = 0;
    int executions = 0;
    std::optional<int> result;
    asyncCall(client, server, 21,
        force_timeout ? std::chrono::seconds(0) : std::chrono::seconds(30),
        [&](int request) {
            check(server.running_in_this_thread(), "Request escaped server strand");
            ++executions;
            return request * 2;
        },
        [&](std::optional<int> response) {
            check(client.running_in_this_thread(), "Response escaped client strand");
            ++completions;
            result = response;
        });
    if (force_timeout) client_io.run(); // Deadline expires before server is driven.
    else client_io.poll();
    server_io.run();
    if (client_io.stopped()) client_io.restart();
    client_io.run();
    check(completions == 1 && executions == 1, "Service did not complete exactly once locally");
    check(force_timeout ? !result : result == 42, "Unexpected service result");
}

struct Parameters {
    int iterations = 20;
    double voxel_size = 1.0;
    unsigned revision = 0;
};
struct ParameterChange {
    int iterations;
    double voxel_size;
    unsigned expected_revision;
};
struct ParameterResult {
    bool applied;
    unsigned revision;
};

void testParameterTransactions() {
    net::io_context io;
    auto client = net::make_strand(io);
    auto owner = net::make_strand(io);
    auto current = std::make_shared<const Parameters>();
    const auto old_snapshot = current;
    std::vector<ParameterResult> responses;
    auto apply = [&](ParameterChange change) {
        check(owner.running_in_this_thread(), "Parameter change escaped its owner");
        if (change.expected_revision != current->revision || change.iterations <= 0
            || !(change.voxel_size > 0)) {
            return ParameterResult{false, current->revision};
        }
        auto candidate = std::make_shared<Parameters>();
        candidate->iterations = change.iterations;
        candidate->voxel_size = change.voxel_size;
        candidate->revision = current->revision + 1;
        current = std::move(candidate);
        return ParameterResult{true, current->revision};
    };
    auto done = [&](std::optional<ParameterResult> result) {
        check(result.has_value(), "Parameter operation unexpectedly timed out");
        responses.push_back(*result);
    };
    asyncCall(client, owner, ParameterChange{30, 2.0, 0}, std::chrono::seconds(30), apply, done);
    asyncCall(client, owner, ParameterChange{0, 3.0, 1}, std::chrono::seconds(30), apply, done);
    asyncCall(client, owner, ParameterChange{40, 4.0, 0}, std::chrono::seconds(30), apply, done);
    io.run();
    check(responses.size() == 3, "Missing parameter response");
    check(responses[0].applied && !responses[1].applied && !responses[2].applied,
          "Invalid or stale parameter transaction was accepted");
    check(current->revision == 1 && current->iterations == 30 && current->voxel_size == 2.0,
          "Parameter transaction partially applied");
    check(old_snapshot->revision == 0 && old_snapshot->iterations == 20,
          "An immutable parameter snapshot changed");
}

int main() {
    try {
        testModuleSerialization();
        testLatestAndReentrancy();
        testService(false);
        testService(true);
        testParameterTransactions();
        std::cout << "PASS: module serialization; Latest coalescing/reentrancy; "
                     "service success; timeout/late reply; parameter transactions\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
