#include <SimpleSLAM/core/infra/callback_slot.hpp>

#include <catch2/catch_test_macros.hpp>
#include <string>
#include <vector>

using namespace simpleslam;

TEST_CASE("CallbackSlot connect 和 emit", "[callback_slot]") {
    CallbackSlot<int> slot;
    REQUIRE(slot.empty());

    std::vector<int> received;
    slot.connect([&](int v) { received.push_back(v); });
    REQUIRE(slot.size() == 1);

    slot.emit(42);
    REQUIRE(received.size() == 1);
    REQUIRE(received[0] == 42);
}

TEST_CASE("CallbackSlot 多个回调按顺序调用", "[callback_slot]") {
    CallbackSlot<int> slot;
    std::string order;
    slot.connect([&](int) { order += "A"; });
    slot.connect([&](int) { order += "B"; });
    slot.connect([&](int) { order += "C"; });

    slot.emit(0);
    REQUIRE(order == "ABC");
}

TEST_CASE("CallbackSlot disconnect 移除指定回调", "[callback_slot]") {
    CallbackSlot<int> slot;
    int count_a = 0, count_b = 0;
    slot.connect([&](int) { ++count_a; });
    auto handle_b = slot.connect([&](int) { ++count_b; });

    slot.emit(0);
    REQUIRE(count_a == 1);
    REQUIRE(count_b == 1);

    slot.disconnect(handle_b);
    slot.emit(0);
    REQUIRE(count_a == 2);
    REQUIRE(count_b == 1);
}

TEST_CASE("CallbackSlot 多参数", "[callback_slot]") {
    CallbackSlot<int, std::string> slot;
    int received_int = 0;
    std::string received_str;
    slot.connect([&](int i, std::string s) {
        received_int = i;
        received_str = s;
    });

    slot.emit(7, "hello");
    REQUIRE(received_int == 7);
    REQUIRE(received_str == "hello");
}
