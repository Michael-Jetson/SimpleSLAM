#include <SimpleSLAM/core/concepts/any_loop_detector.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

using namespace simpleslam;

namespace {

struct AlwaysDetectMock {
    int add_count = 0;
    int detect_count = 0;

    void addKeyframe(const KeyframeData&) { ++add_count; }

    std::vector<LoopCandidate> detect(const KeyframeData&) {
        ++detect_count;
        LoopCandidate candidate;
        candidate.match_keyframe_id = 42;
        candidate.T_match_query = SE3d{};
        candidate.score = 0.95;
        return {candidate};
    }
};

struct NeverDetectMock {
    void addKeyframe(const KeyframeData&) {}
    std::vector<LoopCandidate> detect(const KeyframeData&) { return {}; }
};

}  // namespace

TEST_CASE("AnyLoopDetector 转发 addKeyframe 和 detect", "[any_loop_detector]") {
    AnyLoopDetector detector(AlwaysDetectMock{});

    KeyframeData kf;
    kf.id = 1;
    detector.addKeyframe(kf);

    auto candidates = detector.detect(kf);
    REQUIRE(candidates.size() == 1);
    REQUIRE(candidates[0].match_keyframe_id == 42);
    REQUIRE(candidates[0].score == Catch::Approx(0.95));
}

TEST_CASE("AnyLoopDetector NeverDetect 返回空列表", "[any_loop_detector]") {
    AnyLoopDetector detector(NeverDetectMock{});

    KeyframeData kf;
    auto candidates = detector.detect(kf);
    REQUIRE(candidates.empty());
}

TEST_CASE("AnyLoopDetector move 语义", "[any_loop_detector]") {
    AnyLoopDetector a(AlwaysDetectMock{});
    REQUIRE(a.valid());

    AnyLoopDetector b = std::move(a);
    REQUIRE(b.valid());
    REQUIRE_FALSE(a.valid());

    KeyframeData kf;
    auto candidates = b.detect(kf);
    REQUIRE_FALSE(candidates.empty());
}

TEST_CASE("AnyLoopDetector 不同类型擦除", "[any_loop_detector]") {
    AnyLoopDetector always(AlwaysDetectMock{});
    AnyLoopDetector never(NeverDetectMock{});

    KeyframeData kf;
    REQUIRE_FALSE(always.detect(kf).empty());
    REQUIRE(never.detect(kf).empty());
}

TEST_CASE("AnyLoopDetector 满足 LoopDetector concept", "[any_loop_detector]") {
    static_assert(LoopDetector<AnyLoopDetector>);
    REQUIRE(true);
}
