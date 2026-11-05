// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 打包器的时间契约：三条水位线各管各的、边界样本保证区间可插值、
/// IMU 是能力不是前提、缺口显式标记、附件按自身时刻带容差。

#include <SimpleSLAM/framework/sensor_source.hpp>

#include <catch2/catch_test_macros.hpp>

#include <memory>

using namespace simpleslam::fw;

namespace {

Ptr<ImuSample> imu(std::int64_t ns) {
    auto sample = std::make_shared<ImuSample>();
    sample->time = Timestamp{ns};
    return sample;
}

Ptr<LidarScan> scan(std::int64_t reference_ns) {
    auto s = std::make_shared<LidarScan>();
    s->reference_time = Timestamp{reference_ns};
    s->coverage = TimeRange{Timestamp{reference_ns - 100}, Timestamp{reference_ns}};
    s->points.push(1.0f, 2.0f, 3.0f);
    return s;
}

Ptr<CameraFrame> image(std::int64_t capture_ns) {
    auto f = std::make_shared<CameraFrame>();
    f->capture_time = Timestamp{capture_ns};
    return f;
}

}  // namespace

TEST_CASE("IMU 区间从上次已接纳时刻起算", "[framework][packetizer]") {
    Packetizer packetizer;
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));

    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());
    CHECK(first->sequence == 1);
    CHECK(first->imu.size() == 11);
    CHECK(first->imuCoversDeclaredRange());
    packetizer.notifyAdmitted(first->sequence);
    packetizer.notifyConsumed(first->sequence);

    for (std::int64_t t = 110; t <= 200; t += 10) packetizer.feed(imu(t));
    auto second = packetizer.feed(scan(200));
    REQUIRE(second.has_value());
    // 第二包的区间起点是上一包的参考时刻，不是本次扫描的起点
    CHECK(second->imu_coverage.begin == Timestamp{100});
    CHECK(second->imu_coverage.end == Timestamp{200});
    CHECK(second->imuCoversDeclaredRange());
}

TEST_CASE("被拒的包不推进接纳水位，那段 IMU 能补回来", "[framework][packetizer]") {
    // 生成时就推进水位并移除 IMU 的话，中间一包被拒之后，
    // 下一包会从被拒帧的时刻起算，被拒那一段的 IMU 永久丢失。
    Packetizer packetizer;
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));
    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());
    packetizer.notifyAdmitted(first->sequence);

    for (std::int64_t t = 110; t <= 200; t += 10) packetizer.feed(imu(t));
    auto second = packetizer.feed(scan(200));
    REQUIRE(second.has_value());
    packetizer.notifyRejected(second->sequence);  // 下游满载，拒收

    for (std::int64_t t = 210; t <= 300; t += 10) packetizer.feed(imu(t));
    auto third = packetizer.feed(scan(300));
    REQUIRE(third.has_value());
    CHECK(third->gap == GapReason::kDroppedTrigger);
    CHECK(third->propagation.begin == Timestamp{100});   // 从上次接纳起算，不是从 200
    CHECK(third->imu_coverage.begin == Timestamp{100});
    CHECK(third->imu.size() == 21);                      // 100..300 一条不少
    CHECK(third->supportComplete());
}

TEST_CASE("还没有接纳过任何包时，需求从本帧采集起点算", "[framework][packetizer]") {
    // 首个被接纳的包之前没有状态可传播：需求就是本帧的采集区间，不由现有数据反推
    Packetizer packetizer;
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));
    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());
    packetizer.notifyRejected(first->sequence);

    for (std::int64_t t = 110; t <= 200; t += 10) packetizer.feed(imu(t));
    auto second = packetizer.feed(scan(200));
    REQUIRE(second.has_value());
    CHECK(second->imu_required.begin == Timestamp{100});  // scan(200) 覆盖 [100, 200]
    CHECK(second->imu_coverage.begin == Timestamp{100});
    CHECK(second->supportComplete());
}

TEST_CASE("未确认消费前不回收原始 IMU", "[framework][packetizer]") {
    Packetizer packetizer;
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));
    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());

    packetizer.notifyAdmitted(first->sequence);
    CHECK(packetizer.imuBacklog() == 11);  // 已接纳，但还没人说用完了

    packetizer.notifyConsumed(first->sequence);
    CHECK(packetizer.imuBacklog() == 1);  // 只留一个跨越消费水位的边界样本
    CHECK(packetizer.releasedUntil() == Timestamp{100});
}

TEST_CASE("终点落在两个 IMU 之间时取跨越端点的样本", "[framework][packetizer]") {
    // IMU 在 0/10/20、扫描终点 15 时，只带到 10 的样本会被自身的覆盖检查拒绝。
    // 端点要的是插值，所以必须带上跨过去的那一条。
    Packetizer packetizer;
    packetizer.feed(imu(-100));  // 左括点：扫描 [-85, 15] 的起点之前
    packetizer.feed(imu(0));
    packetizer.feed(imu(10));
    packetizer.feed(imu(20));

    auto packet = packetizer.feed(scan(15));
    REQUIRE(packet.has_value());
    CHECK(packet->imu.size() == 4);
    CHECK(packet->imu_coverage.end == Timestamp{15});
    CHECK(packet->imuCoversDeclaredRange());
    CHECK(packet->gap == GapReason::kNone);
}

TEST_CASE("纯 LiDAR 输入声明不需要 IMU 就能发包", "[framework][packetizer]") {
    PacketizerPolicy policy;
    policy.imu = ImuRequirement::kNotRequired;
    Packetizer packetizer(policy);

    auto packet = packetizer.feed(scan(100));
    REQUIRE(packet.has_value());  // 一条 IMU 也没有，照样发
    CHECK(packet->imu.size() == 0);
    CHECK(packet->imuCoversDeclaredRange());  // 声明的是空区间，不是撒谎的区间
    CHECK(packet->gap == GapReason::kNone);
}

TEST_CASE("丢触发帧时 IMU 并入下一包并标记缺口", "[framework][packetizer]") {
    Packetizer packetizer;
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));
    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());
    packetizer.notifyAdmitted(first->sequence);
    packetizer.notifyConsumed(first->sequence);

    for (std::int64_t t = 110; t <= 150; t += 10) packetizer.feed(imu(t));
    CHECK_FALSE(packetizer.feed(scan(160)).has_value());  // IMU 只到 150，还在等水位线
    CHECK(packetizer.waitingForWatermark());
    packetizer.dropTrigger();  // 过载：丢掉这个尚未发出的触发帧

    for (std::int64_t t = 160; t <= 200; t += 10) packetizer.feed(imu(t));
    auto next = packetizer.feed(scan(200));
    REQUIRE(next.has_value());
    CHECK(next->gap == GapReason::kDroppedTrigger);
    CHECK(next->imu_coverage.begin == Timestamp{100});
    CHECK(next->imu.size() == 11);  // 被丢帧那段的 IMU 一条不少（含保留的边界样本）
}

TEST_CASE("消费水位只沿已确认的连续前缀推进", "[framework][packetizer]") {
    // 接纳时就把账本条目删掉的话，之后消费确认找不到条目，只能拿当前接纳水位顶替——
    // 于是"只确认了第 1 包"会把水位推到第 2 包。
    Packetizer packetizer;
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));
    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());
    packetizer.notifyAdmitted(first->sequence);  // 上一包结案之后才生成下一包
    for (std::int64_t t = 110; t <= 200; t += 10) packetizer.feed(imu(t));
    auto second = packetizer.feed(scan(200));
    REQUIRE(second.has_value());
    packetizer.notifyAdmitted(second->sequence);
    CHECK(packetizer.admittedUntil() == Timestamp{200});
    CHECK(packetizer.inFlight() == 2);  // 接纳不出账

    packetizer.notifyConsumed(first->sequence);
    CHECK(packetizer.releasedUntil() == Timestamp{100});  // 不是 200
    CHECK(packetizer.inFlight() == 1);

    packetizer.notifyConsumed(second->sequence);
    CHECK(packetizer.releasedUntil() == Timestamp{200});
    CHECK(packetizer.inFlight() == 0);

    // 重复确认是幂等的
    packetizer.notifyConsumed(second->sequence);
    CHECK(packetizer.releasedUntil() == Timestamp{200});
}

TEST_CASE("中间一包没确认，后面的确认不能替它作数", "[framework][packetizer]") {
    Packetizer packetizer;
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));
    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());
    packetizer.notifyAdmitted(first->sequence);
    for (std::int64_t t = 110; t <= 200; t += 10) packetizer.feed(imu(t));
    auto second = packetizer.feed(scan(200));
    REQUIRE(second.has_value());
    packetizer.notifyAdmitted(second->sequence);

    packetizer.notifyConsumed(second->sequence);  // 跳过第一包
    CHECK(packetizer.releasedUntil() == Timestamp{0});  // 前缀不连续，一步也不推进

    packetizer.notifyConsumed(first->sequence);
    CHECK(packetizer.releasedUntil() == Timestamp{200});  // 补齐之后一次推到底
}

TEST_CASE("IMU 积压超限是显式缺口而不是无声增长", "[framework][packetizer]") {
    PacketizerPolicy policy;
    policy.max_imu_backlog = 4;
    Packetizer packetizer(policy);

    for (std::int64_t t = 0; t < 10; ++t) packetizer.feed(imu(t));
    CHECK(packetizer.imuBacklog() == 4);
    packetizer.feed(imu(100));  // 越过水位线

    auto packet = packetizer.feed(scan(100));
    REQUIRE(packet.has_value());
    CHECK(packet->gap == GapReason::kImuGap);
}

TEST_CASE("附件按自身采集时刻带容差，不硬塞最近的一帧", "[framework][packetizer]") {
    PacketizerPolicy policy;
    policy.attachment_tolerance = std::chrono::nanoseconds(20);
    Packetizer packetizer(policy);

    packetizer.feed(imu(0));
    packetizer.feed(imu(100));    // 越过水位线
    packetizer.feed(image(90));   // 在容差内
    packetizer.feed(image(500));  // 远在容差外

    auto packet = packetizer.feed(scan(100));
    REQUIRE(packet.has_value());
    REQUIRE(packet->attachments.images.size() == 1);
    CHECK(packet->attachments.images.front()->capture_time == Timestamp{90});
}

TEST_CASE("相机触发时结构相同", "[framework][packetizer]") {
    PacketizerPolicy policy;
    policy.trigger_on_lidar = false;
    Packetizer packetizer(policy);

    packetizer.feed(imu(0));
    packetizer.feed(imu(50));
    auto packet = packetizer.feed(image(50));
    REQUIRE(packet.has_value());
    CHECK_FALSE(packet->triggeredByLidar());
    REQUIRE(packet->camera() != nullptr);
    CHECK(packet->camera()->capture_time == Timestamp{50});
}

TEST_CASE("水位线未越过时不发包", "[framework][packetizer]") {
    PacketizerPolicy policy;
    policy.post_support = std::chrono::nanoseconds(0);
    Packetizer packetizer(policy);
    packetizer.feed(imu(0));
    packetizer.feed(imu(50));

    // 扫描终点 100，但 IMU 只到 50：不能发，否则这一包支撑不了去畸变
    CHECK_FALSE(packetizer.feed(scan(100)).has_value());
    CHECK(packetizer.waitingForWatermark());

    auto packet = packetizer.feed(imu(100));
    REQUIRE(packet.has_value());
    CHECK(packet->imuCoversDeclaredRange());
    CHECK_FALSE(packetizer.waitingForWatermark());
}

TEST_CASE("时间偏置支撑区间要等到参考时刻之后的样本", "[framework][packetizer]") {
    PacketizerPolicy policy;
    policy.post_support = std::chrono::nanoseconds(30);
    Packetizer packetizer(policy);
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));

    CHECK_FALSE(packetizer.feed(scan(100)).has_value());  // 只到 100，支撑到 130 才够
    packetizer.feed(imu(110));
    packetizer.feed(imu(120));
    CHECK(packetizer.waitingForWatermark());

    auto packet = packetizer.feed(imu(130));
    REQUIRE(packet.has_value());
    CHECK(packet->imu_coverage.end == Timestamp{130});
    CHECK(packet->time_offset_support == std::chrono::nanoseconds(30));
    CHECK(packet->imuCoversDeclaredRange());
}

TEST_CASE("数据结束时缺支撑的触发帧如实交代", "[framework][packetizer]") {
    Packetizer packetizer;
    packetizer.feed(imu(0));
    packetizer.feed(imu(50));
    CHECK_FALSE(packetizer.feed(scan(100)).has_value());

    auto tail = packetizer.flush();
    REQUIRE(tail.has_value());  // 不丢最后一帧，而是标明它缺支撑
    CHECK(tail->gap == GapReason::kImuGap);
    CHECK(tail->imu_coverage.end == Timestamp{50});
    CHECK(tail->imuCoversDeclaredRange());  // 声明的区间与实际带的样本一致
    CHECK_FALSE(packetizer.waitingForWatermark());
}

TEST_CASE("账本超限之后回收水位就地失同步", "[framework][packetizer]") {
    // 丢掉的是"消费与否的证据"。之后账本里挨着的两条中间其实缺了一段，
    // 只看"front 已确认"就跨过去，等于把"只沿已证明完成的连续前缀推进"说成空话。
    PacketizerPolicy policy;
    policy.imu = ImuRequirement::kNotRequired;
    policy.max_pending_packets = 1;
    Packetizer packetizer(policy);

    auto scan = [](std::int64_t ms) {
        auto s = std::make_shared<LidarScan>();
        s->reference_time = Timestamp{ms * 1'000'000};
        s->coverage = TimeRange{Timestamp{(ms - 100) * 1'000'000}, s->reference_time};
        return SensorDatum{Ptr<LidarScan>(s)};
    };

    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());
    packetizer.notifyAdmitted(first->sequence);
    auto second = packetizer.feed(scan(200));  // 账本容量 1，第 1 包被挤掉
    REQUIRE(second.has_value());
    packetizer.notifyAdmitted(second->sequence);
    CHECK(packetizer.ledgerOverflows() == 1);
    CHECK(packetizer.reclamationDesynchronized());

    packetizer.notifyConsumed(second->sequence);
    // 第 1 包从未被确认消费过，水位不许跳到第 2 包
    CHECK(packetizer.releasedUntil().count() == 0);

    packetizer.resyncReclamation();  // 显式认下这个缺口
    packetizer.notifyConsumed(second->sequence);
    CHECK(packetizer.releasedUntil() == Timestamp{200 * 1'000'000});
}

TEST_CASE("被拒的序号在连续前缀里可以跳过", "[framework][packetizer]") {
    // 被拒的包从未交付，下游手里没有它——与"交付了但证据没了"不是一回事
    PacketizerPolicy policy;
    policy.imu = ImuRequirement::kNotRequired;
    Packetizer packetizer(policy);

    auto scan = [](std::int64_t ms) {
        auto s = std::make_shared<LidarScan>();
        s->reference_time = Timestamp{ms * 1'000'000};
        s->coverage = TimeRange{Timestamp{(ms - 100) * 1'000'000}, s->reference_time};
        return SensorDatum{Ptr<LidarScan>(s)};
    };

    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());
    packetizer.notifyRejected(first->sequence);
    auto second = packetizer.feed(scan(200));
    REQUIRE(second.has_value());
    packetizer.notifyAdmitted(second->sequence);
    packetizer.notifyConsumed(second->sequence);

    CHECK_FALSE(packetizer.reclamationDesynchronized());
    CHECK(packetizer.releasedUntil() == Timestamp{200 * 1'000'000});
}
