// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 服务：终态枚举、响应回到调用方上下文、超时与迟到响应只完成一次、
/// 准入上限、运行时线程内阻塞调用被拒绝。

#include <SimpleSLAM/comm/runtime.hpp>

#include <catch2/catch_test_macros.hpp>

#include <optional>
#include <stdexcept>
#include <string>
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
    int calls = 0;
    bool block_forever = false;

    void onInit() override {
        advertiseService<AddRequest, AddResponse>("~add", &MathServer::onAdd, /*max_in_flight=*/1);
    }

    AddResponse onAdd(const AddRequest& request) {
        ++calls;
        if (request.a < 0) throw std::runtime_error("不接受负数");
        return AddResponse{request.a + request.b};
    }
};

class MathClient final : public Module {
public:
    std::vector<ServiceResult<AddResponse>> results;
    bool saw_own_context = true;

    void onInit() override { client_ = serviceClient<AddRequest, AddResponse>("/server/add"); }

    ServiceStatus send(AddRequest request, Duration timeout) {
        auto* self = this;
        return client_.asyncCall(request, timeout, [self](ServiceResult<AddResponse> r) {
            self->saw_own_context = self->saw_own_context && self->executor().runningInThisThread();
            self->results.push_back(std::move(r));
        });
    }

    ServiceClient<AddRequest, AddResponse>& client() { return client_; }

private:
    ServiceClient<AddRequest, AddResponse> client_;
};

}  // namespace

TEST_CASE("服务调用成功，响应回到调用方上下文", "[comm][service]") {
    Runtime runtime(RuntimeOptions{1, /*virtual_clock=*/true});
    auto& server = runtime.addModule<MathServer>("/server");
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.init();

    client.executor().post([&] { client.send(AddRequest{20, 22}, Duration{0}); });
    runtime.spinOnce();
    runtime.spinOnce();
    runtime.spinOnce();

    REQUIRE(client.results.size() == 1);
    CHECK(client.results[0].status == ServiceStatus::kOk);
    CHECK(client.results[0].value->sum == 42);
    CHECK(client.saw_own_context);
    CHECK(server.calls == 1);
}

TEST_CASE("没有服务端时在入口返回 NoProvider，不回调", "[comm][service]") {
    // 不受理就不欠一个终态：拒绝是同步可见的返回值，不另外回调
    Runtime runtime(RuntimeOptions{1, true});
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.init();

    CHECK_FALSE(client.client().valid());
    ServiceStatus admission = ServiceStatus::kOk;
    client.executor().post([&] { admission = client.send(AddRequest{1, 2}, Duration{0}); });
    for (int i = 0; i < 4; ++i) runtime.spinOnce();

    CHECK(admission == ServiceStatus::kNoProvider);
    CHECK(client.results.empty());
}

TEST_CASE("服务端抛异常映射为 HandlerError", "[comm][service]") {
    Runtime runtime(RuntimeOptions{1, true});
    runtime.addModule<MathServer>("/server");
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.setErrorReporter([](std::string_view, std::string_view) {});
    runtime.init();

    client.executor().post([&] { client.send(AddRequest{-1, 2}, Duration{0}); });
    for (int i = 0; i < 4; ++i) runtime.spinOnce();

    REQUIRE(client.results.size() == 1);
    CHECK(client.results[0].status == ServiceStatus::kHandlerError);
    CHECK_FALSE(client.results[0].value.has_value());
}

TEST_CASE("超时后迟到的响应不再完成第二次", "[comm][service]") {
    Runtime runtime(RuntimeOptions{1, true});
    auto& server = runtime.addModule<MathServer>("/server");
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.init();

    // 只推进虚拟时间、不处理服务端任务，让超时先发生
    client.executor().post([&] { client.send(AddRequest{1, 2}, std::chrono::milliseconds(10)); });
    runtime.spinOnce(1);  // 只跑 send 本身
    runtime.virtualClock()->advanceBy(std::chrono::milliseconds(20));
    for (int i = 0; i < 6; ++i) runtime.spinOnce();

    REQUIRE(client.results.size() == 1);
    CHECK(client.results[0].status == ServiceStatus::kTimeout);
    // 服务端其实执行了：超时不撤销副作用
    CHECK(server.calls == 1);

    for (int i = 0; i < 6; ++i) runtime.spinOnce();
    CHECK(client.results.size() == 1);  // 迟到响应被丢弃，不产生第二个终态
}

TEST_CASE("在飞请求超过上限时拒绝", "[comm][service]") {
    Runtime runtime(RuntimeOptions{1, true});
    runtime.addModule<MathServer>("/server");
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.setErrorReporter([](std::string_view, std::string_view) {});
    runtime.init();

    client.executor().post([&] {
        client.send(AddRequest{1, 1}, Duration{0});
        client.send(AddRequest{2, 2}, Duration{0});  // max_in_flight = 1
    });
    runtime.spinOnce(1);
    for (int i = 0; i < 6; ++i) runtime.spinOnce();

    REQUIRE(client.results.size() == 2);
    const bool has_rejected = client.results[0].status == ServiceStatus::kRejected
                              || client.results[1].status == ServiceStatus::kRejected;
    CHECK(has_rejected);
}

TEST_CASE("运行时线程内做阻塞调用会被入口拒绝", "[comm][service]") {
    Runtime runtime(RuntimeOptions{1, true});
    runtime.addModule<MathServer>("/server");
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.init();

    std::optional<ServiceStatus> status;
    client.executor().post([&] {
        status = client.client().call(AddRequest{1, 2}, std::chrono::milliseconds(1)).status;
    });
    runtime.spinOnce();

    REQUIRE(status.has_value());
    CHECK(*status == ServiceStatus::kWouldDeadlock);
}

TEST_CASE("模块关闭之后的调用在入口可见地失败", "[comm][service]") {
    // 终态的承诺是"至多一次"：收件箱关了就投不进业务回调。
    // 所以关闭之后再发起的调用必须在入口同步拒绝，而不是先受理再静默丢。
    Runtime runtime(RuntimeOptions{1, /*virtual_clock=*/true});
    runtime.addModule<MathServer>("/server");
    auto& client = runtime.addModule<MathClient>("/client");
    runtime.init();

    client.executor().post([&] { client.send(AddRequest{1, 2}, Duration{0}); });
    runtime.spinOnce();
    runtime.spinOnce();
    runtime.spinOnce();
    REQUIRE(client.results.size() == 1);

    client.shutdown();  // 只关调用方模块，运行时还在
    CHECK(client.send(AddRequest{3, 4}, Duration{0}) == ServiceStatus::kStopped);
    runtime.spinOnce();
    CHECK(client.results.size() == 1);  // 没有凭空多出一个终态
}
