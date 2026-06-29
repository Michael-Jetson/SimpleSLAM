#include <SimpleSLAM/core/infra/service.hpp>

#include <catch2/catch_test_macros.hpp>
#include <stdexcept>
#include <string>

using namespace simpleslam;

namespace {
struct AddReq {
    int a;
    int b;
};
}  // namespace

TEST_CASE("Service 基本 request/reply", "[service]") {
    ServiceRegistry reg;
    auto srv = reg.advertiseService<AddReq, int>(
        "add", [](const AddReq& r) { return r.a + r.b; });
    REQUIRE(reg.hasService("add"));

    const int result = reg.call<AddReq, int>("add", {3, 4});
    REQUIRE(result == 7);
}

TEST_CASE("Service 未注册抛异常", "[service]") {
    ServiceRegistry reg;
    REQUIRE_THROWS(reg.call<int, int>("missing", 1));
}

TEST_CASE("Service 类型不匹配抛异常", "[service]") {
    ServiceRegistry reg;
    auto srv = reg.advertiseService<int, int>("svc", [](const int& x) { return x * 2; });
    REQUIRE_THROWS(reg.call<std::string, int>("svc", std::string("bad")));
}

TEST_CASE("Service 重复 advertise 抛异常", "[service]") {
    ServiceRegistry reg;
    auto a = reg.advertiseService<int, int>("dup", [](const int& x) { return x; });
    REQUIRE_THROWS(reg.advertiseService<int, int>("dup", [](const int& x) { return x; }));
}

TEST_CASE("ServiceServer RAII 注销", "[service]") {
    ServiceRegistry reg;
    {
        auto srv = reg.advertiseService<int, int>("tmp", [](const int& x) { return x; });
        REQUIRE(reg.hasService("tmp"));
    }
    REQUIRE_FALSE(reg.hasService("tmp"));  // 句柄析构 → 自动注销
}

TEST_CASE("Service handler 抛异常传播给调用者（不隔离）", "[service]") {
    ServiceRegistry reg;
    auto srv = reg.advertiseService<int, int>(
        "boom", [](const int&) -> int { throw std::runtime_error("svc fail"); });

    bool caught_runtime = false;
    try {
        reg.call<int, int>("boom", 1);
    } catch (const std::runtime_error&) {
        caught_runtime = true;  // 按类型传播，未被吞（区别于 pub/sub 隔离）
    }
    REQUIRE(caught_runtime);
}

TEST_CASE("ServiceClient typed 客户端", "[service]") {
    ServiceRegistry reg;
    auto srv = reg.advertiseService<AddReq, int>(
        "add2", [](const AddReq& r) { return r.a * r.b; });

    ServiceClient<AddReq, int> client("add2", &reg);
    REQUIRE(client.exists());
    REQUIRE(client.call({6, 7}) == 42);
}
