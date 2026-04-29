#include <SimpleSLAM/core/concepts/any_registration_target.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace simpleslam;

namespace {

struct MockMapTarget {
    int match_count = 0;
    int update_count = 0;
    size_t point_count = 0;

    void match(const LidarScan&, const SE3d&, MatchResult& result) {
        ++match_count;
        result.num_valid = 10;
    }

    void update(const LidarScan&, const SE3d&) {
        ++update_count;
        point_count += 100;
    }

    [[nodiscard]] bool empty() const { return point_count == 0; }
    [[nodiscard]] size_t size() const { return point_count; }
};

struct AnotherMockTarget {
    void match(const LidarScan&, const SE3d&, MatchResult& result) {
        result.num_valid = 42;
    }
    void update(const LidarScan&, const SE3d&) {}
    [[nodiscard]] bool empty() const { return true; }
    [[nodiscard]] size_t size() const { return 0; }
};

}  // namespace

TEST_CASE("AnyRegistrationTarget 包装 mock 并转发 match", "[any_reg_target]") {
    MockMapTarget mock;
    AnyRegistrationTarget target(std::move(mock));

    LidarScan scan;
    SE3d pose;
    MatchResult result;

    target.match(scan, pose, result);
    REQUIRE(result.num_valid == 10);
}

TEST_CASE("AnyRegistrationTarget 包装 mock 并转发 update", "[any_reg_target]") {
    AnyRegistrationTarget target(MockMapTarget{});

    LidarScan scan;
    SE3d pose;

    REQUIRE(target.empty());
    REQUIRE(target.size() == 0);

    target.update(scan, pose);

    REQUIRE_FALSE(target.empty());
    REQUIRE(target.size() == 100);
}

TEST_CASE("AnyRegistrationTarget empty 和 size 转发", "[any_reg_target]") {
    AnyRegistrationTarget target(MockMapTarget{});
    REQUIRE(target.empty());
    REQUIRE(target.size() == 0);

    LidarScan scan;
    SE3d pose;
    target.update(scan, pose);
    target.update(scan, pose);

    REQUIRE(target.size() == 200);
    REQUIRE_FALSE(target.empty());
}

TEST_CASE("AnyRegistrationTarget move 语义", "[any_reg_target]") {
    AnyRegistrationTarget a(MockMapTarget{});
    REQUIRE(a.valid());

    AnyRegistrationTarget b = std::move(a);
    REQUIRE(b.valid());
    REQUIRE_FALSE(a.valid());

    LidarScan scan;
    SE3d pose;
    MatchResult result;
    b.match(scan, pose, result);
    REQUIRE(result.num_valid == 10);
}

TEST_CASE("AnyRegistrationTarget 不同类型擦除", "[any_reg_target]") {
    AnyRegistrationTarget target_a(MockMapTarget{});
    AnyRegistrationTarget target_b(AnotherMockTarget{});

    LidarScan scan;
    SE3d pose;
    MatchResult result_a, result_b;

    target_a.match(scan, pose, result_a);
    target_b.match(scan, pose, result_b);

    REQUIRE(result_a.num_valid == 10);
    REQUIRE(result_b.num_valid == 42);
}

TEST_CASE("AnyRegistrationTarget 满足 RegistrationTarget concept", "[any_reg_target]") {
    static_assert(RegistrationTarget<AnyRegistrationTarget>);
    REQUIRE(true);
}
