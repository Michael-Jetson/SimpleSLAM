// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 服务：终态枚举、响应回到调用方线程、超时与迟到响应只结案一次、运行时线程内阻塞调用被拒绝。

#include <SimpleSLAM/comm/runtime.hpp>

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace simpleslam::comm;

namespace {

struct AddRequest {
    int a = 0;
    int b = 0;
};
struct AddResponse {
    int sum = 0;
};

class MathServer final : public Module {
public:
    std::atomic<int> calls{0};
    void onInit() override { advertiseService("~add", &MathServer::onAdd); }
    AddResponse onAdd(const AddRequest& request) {
        ++calls;
        if (request.a < 0) throw std::runtime_error("不接受负数");
        return AddResponse{request.a + request.b};
    }
};

class MathClient final : public Module {
public:
    std::vector<ServiceResult<AddResponse>> results;
    std::thread::id own_thread;
    std::thread::id completion_thread;
    std::atomic<int> completions{0};

    void onInit() override { client_ = serviceClient<AddRequest, AddResponse>("/server/add"); }
    void onStart() override { own_thread = std::this_thread::get_id(); }

    ServiceStatus send(AddRequest request, Duration timeout) {
        return client_.asyncCall(request, timeout, [this](ServiceResult<AddResponse> r) {
            completion_thread = std::this_thread::get_id();
            results.push_back(std::move(r));
            ++completions;
        });
    }
    ServiceClient<AddRequest, AddResponse>& client() { return client_; }

private:
    ServiceClient<AddRequest, AddResponse> client_;
};

}  // namespace

TEST_CASE("服务调用成功，响应回到调用方线程", "[comm][service]") {
    Runtime runtime;
    auto& server = runtime.addModule<MathServer>("/server");
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.init();
    runtime.start();

    REQUIRE(client.mailbox()->post([&] { client.send(AddRequest{20, 22}, Duration{0}); }));
    for (int i = 0; i < 5000 && client.completions.load() == 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    runtime.shutdown();

    REQUIRE(client.results.size() == 1);
    CHECK(client.results[0].status == ServiceStatus::kOk);
    CHECK(client.results[0].get().sum == 42);
    CHECK(client.completion_thread == client.own_thread);
    CHECK(server.calls.load() == 1);
}

TEST_CASE("没有服务端时在入口返回 NoProvider，不回调", "[comm][service]") {
    // 不受理就不欠一个终态：拒绝是同步可见的返回值，不另外回调
    Runtime runtime(RuntimeOptions{.offline = true});
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.init();

    CHECK(client.send(AddRequest{1, 2}, Duration{0}) == ServiceStatus::kNoProvider);
    runtime.waitIdle();
    CHECK(client.results.empty());
}

TEST_CASE("服务端抛异常映射为 HandlerError，原因带回调用方", "[comm][service]") {
    Runtime runtime(RuntimeOptions{.offline = true});
    runtime.addModule<MathServer>("/server");
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.setErrorReporter([](std::string_view, std::string_view) {});
    runtime.init();

    REQUIRE(client.send(AddRequest{-1, 2}, Duration{0}) == ServiceStatus::kOk);
    runtime.waitIdle();

    REQUIRE(client.results.size() == 1);
    CHECK(client.results[0].status == ServiceStatus::kHandlerError);
    CHECK(client.results[0].value == nullptr);
    CHECK(client.results[0].error == "不接受负数");
    CHECK_THROWS_AS(client.results[0].get(), ServiceError);
}

TEST_CASE("超时后迟到的响应不再结案第二次", "[comm][service]") {
    Runtime runtime(RuntimeOptions{.offline = true});
    auto& server = runtime.addModule<MathServer>("/server");
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.init();

    // 只推进虚拟时间、不处理服务端任务，让超时先发生
    REQUIRE(client.send(AddRequest{1, 2}, std::chrono::milliseconds(10)) == ServiceStatus::kOk);
    runtime.virtualClock()->advanceTo(std::chrono::milliseconds(20));
    runtime.waitIdle();

    REQUIRE(client.results.size() == 1);
    CHECK(client.results[0].status == ServiceStatus::kTimeout);
    CHECK(server.calls.load() == 1);  // 服务端其实执行了：超时不撤销副作用
}

TEST_CASE("运行时线程内做阻塞调用会被入口拒绝", "[comm][service]") {
    Runtime runtime(RuntimeOptions{.offline = true});
    runtime.addModule<MathServer>("/server");
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.init();

    std::optional<ServiceStatus> status;
    REQUIRE(client.mailbox()->post([&] {
        status = client.client().call(AddRequest{1, 2}, std::chrono::milliseconds(1)).status;
    }));
    runtime.waitIdle();

    REQUIRE(status.has_value());
    CHECK(*status == ServiceStatus::kWouldDeadlock);
}

TEST_CASE("外部线程可以阻塞调用，不需要收件箱", "[comm][service]") {
    Runtime runtime;
    runtime.addModule<MathServer>("/server");
    runtime.init();
    runtime.start();

    auto client = runtime.serviceClient<AddRequest, AddResponse>("/server/add");
    CHECK(client.call(AddRequest{2, 3}, std::chrono::seconds(5)).get().sum == 5);
    // 没有收件箱就没法收完成回调，异步调用在入口拒绝
    CHECK(client.asyncCall(AddRequest{}, Duration{0}, nullptr) == ServiceStatus::kStopped);

    runtime.shutdown();
    CHECK(client.call(AddRequest{2, 3}, std::chrono::milliseconds(1)).status == ServiceStatus::kStopped);
}

TEST_CASE("运行结束之后的调用在入口可见地失败", "[comm][service]") {
    Runtime runtime(RuntimeOptions{.offline = true});
    runtime.addModule<MathServer>("/server");
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.init();

    REQUIRE(client.send(AddRequest{1, 2}, Duration{0}) == ServiceStatus::kOk);
    runtime.waitIdle();
    REQUIRE(client.results.size() == 1);

    runtime.shutdown();
    CHECK(client.send(AddRequest{3, 4}, Duration{0}) == ServiceStatus::kStopped);
    CHECK(client.results.size() == 1);  // 没有凭空多出一个终态
}

TEST_CASE("取消时排队的请求结案为 Stopped，完成回调不再执行", "[comm][service]") {
    Runtime runtime(RuntimeOptions{.offline = true});
    auto& server = runtime.addModule<MathServer>("/server");
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.init();

    REQUIRE(client.send(AddRequest{1, 2}, Duration{0}) == ServiceStatus::kOk);
    runtime.shutdown(ShutdownMode::kCancel);  // 服务端的任务没开始就被丢弃

    CHECK(server.calls.load() == 0);
    CHECK(client.results.empty());  // 调用方也关了，结案投不进去，不会在错误的线程上回调
    CHECK(runtime.cancelled());
}
