// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 资源：单写者由类型保证、快照不可变、写失败时强异常保证。

#include <SimpleSLAM/framework/resources.hpp>

#include <catch2/catch_test_macros.hpp>

#include <stdexcept>
#include <string>
#include <vector>

using namespace simpleslam::fw;

namespace {
struct Payload {
    int counter = 0;
    std::vector<std::string> log;
};
}  // namespace

TEST_CASE("一个资源只发一个写令牌", "[framework][resources]") {
    Resource<Payload> resource;
    auto token = resource.acquireWriteToken();
    CHECK(token.valid());
    CHECK_THROWS_AS(resource.acquireWriteToken(), std::logic_error);

    // 令牌销毁后可以再发（模块重启的场景）
    { auto moved = std::move(token); }
    CHECK_NOTHROW(resource.acquireWriteToken());
}

TEST_CASE("已取到的快照不随后续提交改变", "[framework][resources]") {
    Resource<Payload> resource;
    auto token = resource.acquireWriteToken();

    const auto first = resource.snapshot();
    CHECK(resource.version() == 0);

    token.modify([](Payload& draft) {
        draft.counter = 42;
        draft.log.emplace_back("first");
    });

    CHECK(resource.version() == 1);
    CHECK(first->counter == 0);              // 老快照不变
    CHECK(resource.snapshot()->counter == 42);
}

TEST_CASE("修改过程抛异常时资源保持上一个完整版本", "[framework][resources]") {
    Resource<Payload> resource(Payload{7, {"init"}});
    auto token = resource.acquireWriteToken();

    CHECK_THROWS_AS(token.modify([](Payload& draft) {
                        draft.counter = 99;
                        throw std::runtime_error("写到一半失败");
                    }),
                    std::runtime_error);

    CHECK(resource.version() == 0);
    CHECK(resource.snapshot()->counter == 7);
    CHECK(resource.snapshot()->log.size() == 1);
}

TEST_CASE("失效的写令牌不能再写", "[framework][resources]") {
    Resource<Payload> resource;
    WriteToken<Payload> moved;
    {
        auto token = resource.acquireWriteToken();
        moved = std::move(token);
        CHECK_FALSE(token.valid());  // NOLINT(bugprone-use-after-move)
        CHECK_THROWS_AS(token.modify([](Payload&) {}), std::logic_error);
    }
    CHECK(moved.valid());
    CHECK(moved.commit(Payload{1, {}}) == 1);
}
