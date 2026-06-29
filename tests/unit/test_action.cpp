#include <SimpleSLAM/core/infra/comm/action.hpp>

#include <catch2/catch_test_macros.hpp>
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>

using namespace simpleslam;
using namespace std::chrono_literals;

TEST_CASE("Action 基本：sendGoal→执行→反馈→结果", "[action]") {
    ActionRegistry reg;
    auto srv = reg.advertiseActionImpl<int, int, int>(  // Goal=int(N) Feedback=int(进度) Result=int(和)
        "count", [](const int& n, ActionContext<int, int>& ctx) {
            int sum = 0;
            for (int i = 1; i <= n; ++i) {
                if (ctx.isCancelRequested()) break;
                sum += i;
                ctx.publishFeedback(i);
            }
            return sum;
        });
    REQUIRE(reg.hasActionImpl("count"));

    std::atomic<int> last_fb{0};
    auto h = reg.sendGoalImpl<int, int, int>("count", 5,
                                         [&](const int& fb) { last_fb.store(fb); });
    REQUIRE(h.get() == 15);        // 1+2+3+4+5
    REQUIRE(last_fb.load() == 5);  // 最后一次反馈
}

TEST_CASE("Action 协作式取消", "[action]") {
    ActionRegistry reg;
    std::atomic<bool> started{false};
    auto srv = reg.advertiseActionImpl<int, int, int>(
        "longrun", [&](const int&, ActionContext<int, int>& ctx) {
            started.store(true);
            int iters = 0;
            while (!ctx.isCancelRequested() && iters < 1000000) {
                ++iters;
                std::this_thread::sleep_for(100us);
            }
            return iters;
        });

    auto h = reg.sendGoalImpl<int, int, int>("longrun", 0);
    while (!started.load()) std::this_thread::sleep_for(100us);
    h.cancel();
    const int r = h.get();      // 取消后应较快返回
    REQUIRE(r < 1000000);       // 没跑满 = 被取消
}

TEST_CASE("Action handler 异常透传给 get()", "[action]") {
    ActionRegistry reg;
    auto srv = reg.advertiseActionImpl<int, int, int>(
        "boom", [](const int&, ActionContext<int, int>&) -> int {
            throw std::runtime_error("action fail");
        });
    auto h = reg.sendGoalImpl<int, int, int>("boom", 1);

    bool caught = false;
    try {
        h.get();
    } catch (const std::runtime_error&) {
        caught = true;
    }
    REQUIRE(caught);
}

TEST_CASE("Action 未注册 sendGoal 抛异常", "[action]") {
    ActionRegistry reg;
    REQUIRE_THROWS(reg.sendGoalImpl<int, int, int>("missing", 1));
}

TEST_CASE("ActionServer RAII 注销", "[action]") {
    ActionRegistry reg;
    {
        auto srv = reg.advertiseActionImpl<int, int, int>(
            "tmp", [](const int&, ActionContext<int, int>&) { return 0; });
        REQUIRE(reg.hasActionImpl("tmp"));
    }
    REQUIRE_FALSE(reg.hasActionImpl("tmp"));  // 句柄析构 → 自动注销
}

TEST_CASE("Action 全局静态糖（懒加载，免 instance()）", "[action]") {
    auto srv = ActionRegistry::advertiseAction<int, int, int>(
        "g_sum", [](const int& n, ActionContext<int, int>& ctx) {
            int s = 0;
            for (int i = 1; i <= n; ++i) {
                if (ctx.isCancelRequested()) break;
                s += i;
            }
            return s;
        });
    REQUIRE(ActionRegistry::hasAction("g_sum"));
    auto h = ActionRegistry::sendGoal<int, int, int>("g_sum", 4);
    REQUIRE(h.get() == 10);  // 1+2+3+4
}
