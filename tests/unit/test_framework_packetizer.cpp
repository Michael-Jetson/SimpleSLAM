// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 打包器的时间契约：回收跟着生成水位走、过载只丢触发帧、边界样本保证区间可插值、
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

TEST_CASE("IMU 区间从上一包的参考时刻起算", "[framework][packetizer]") {
    Packetizer packetizer;
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));

    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());
    CHECK(first->sequence == 1);
    CHECK(first->imu.size() == 11);
    CHECK(first->imuCoversDeclaredRange());

    for (std::int64_t t = 110; t <= 200; t += 10) packetizer.feed(imu(t));
    auto second = packetizer.feed(scan(200));
    REQUIRE(second.has_value());
    // 第二包的区间起点是上一包的参考时刻，不是本次扫描的起点
    CHECK(second->imu_coverage.begin == Timestamp{100});
    CHECK(second->imu_coverage.end == Timestamp{200});
    CHECK(second->imuCoversDeclaredRange());
}

TEST_CASE("首包的需求从本帧采集起点算", "[framework][packetizer]") {
    // 首包之前没有状态可传播：需求就是本帧的采集区间，不由现有数据反推
    Packetizer packetizer;
    for (std::int64_t t = 50; t <= 100; t += 10) packetizer.feed(imu(t));
    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());
    CHECK(first->imu_required.begin == Timestamp{0});  // scan(100) 覆盖 [0, 100]
    CHECK(first->imu_coverage.begin == Timestamp{50});  // 实际只有这么多，如实收窄
    CHECK(first->gap == GapReason::kImuGap);
}

TEST_CASE("处理进度是累计的：后面的确认把前面的一并算上", "[framework][packetizer]") {
    Packetizer packetizer;
    for (std::int64_t t = 0; t <= 300; t += 10) packetizer.feed(imu(t));
    REQUIRE(packetizer.feed(scan(100)).has_value());
    REQUIRE(packetizer.feed(scan(200)).has_value());
    REQUIRE(packetizer.feed(scan(300)).has_value());
    CHECK(packetizer.inFlight() == 3);

    packetizer.notifyConsumedThrough(2);
    CHECK(packetizer.inFlight() == 1);
    packetizer.notifyConsumedThrough(1);  // 旧的进度晚到，不会倒退
    CHECK(packetizer.inFlight() == 1);
    packetizer.notifyConsumedThrough(99);  // 没发过的序号不算数
    CHECK(packetizer.inFlight() == 0);

    // 超前的进度也不会让之后发出的包被算成已处理
    for (std::int64_t t = 310; t <= 400; t += 10) packetizer.feed(imu(t));
    REQUIRE(packetizer.feed(scan(400)).has_value());
    CHECK(packetizer.inFlight() == 1);
}

TEST_CASE("生成之后就回收原始 IMU：包拷走了自己的那一份，不用等下游", "[framework][packetizer]") {
    Packetizer packetizer;
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));
    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());
    CHECK(first->imu.size() == 11);
    CHECK(packetizer.imuBacklog() == 1);  // 只留一个跨越生成水位的边界样本
    CHECK(packetizer.inFlight() == 1);    // 下游还没处理完，不影响回收
}

TEST_CASE("pre_support 多留一段历史，给起点早于上一包的扫描用", "[framework][packetizer]") {
    PacketizerPolicy policy;
    policy.pre_support = Duration{50};
    Packetizer packetizer(policy);
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));
    REQUIRE(packetizer.feed(scan(100)).has_value());
    CHECK(packetizer.imuBacklog() == 6);  // 50 到 100

    // 下一帧的采集区间 [70, 170] 起点早于上一包的参考时刻 100，那段 IMU 还在
    for (std::int64_t t = 110; t <= 170; t += 10) packetizer.feed(imu(t));
    auto next = packetizer.feed(scan(170));
    REQUIRE(next.has_value());
    CHECK(next->imu_required.begin == Timestamp{70});
    CHECK(next->imu_coverage.begin == Timestamp{70});
    CHECK(next->gap == GapReason::kNone);
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

TEST_CASE("在途达到上限时丢触发帧：IMU 并入下一包，不算缺支撑", "[framework][packetizer]") {
    PacketizerPolicy policy;
    policy.max_in_flight = 1;
    Packetizer packetizer(policy);
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));
    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());
    CHECK(packetizer.inFlight() == 1);

    // 下游还没处理完第一包：这一帧凑齐了括点也不发
    for (std::int64_t t = 110; t <= 200; t += 10) packetizer.feed(imu(t));
    CHECK_FALSE(packetizer.feed(scan(200)).has_value());
    CHECK(packetizer.droppedTriggers() == 1);
    CHECK_FALSE(packetizer.waitingForWatermark());  // 丢掉了，不是还在等
    CHECK(packetizer.generatedUntil() == Timestamp{100});  // 生成水位不动

    packetizer.notifyConsumedThrough(first->sequence);
    for (std::int64_t t = 210; t <= 300; t += 10) packetizer.feed(imu(t));
    auto next = packetizer.feed(scan(300));
    REQUIRE(next.has_value());
    CHECK(next->sequence == 2);  // 序号连续：丢的是帧，不是包
    CHECK(next->gap == GapReason::kNone);
    CHECK(next->dropped_triggers == 1);
    CHECK(next->propagation.begin == Timestamp{100});
    CHECK(next->imu_coverage.begin == Timestamp{100});
    CHECK(next->imu_coverage.end == Timestamp{300});
    CHECK(next->imu.size() == 21);  // 被丢帧那段的 IMU 一条不少
    CHECK(next->supportComplete());
    CHECK(next->imuCoversDeclaredRange());
    CHECK(next->imu_max_gap == Duration{10});  // 中间连续

    // 再下一包不再带这个计数
    packetizer.notifyConsumedThrough(next->sequence);
    for (std::int64_t t = 310; t <= 400; t += 10) packetizer.feed(imu(t));
    auto after = packetizer.feed(scan(400));
    REQUIRE(after.has_value());
    CHECK(after->dropped_triggers == 0);
}

TEST_CASE("一直过载时丢掉的帧攒着 IMU，积压仍然有界", "[framework][packetizer]") {
    PacketizerPolicy policy;
    policy.max_in_flight = 1;
    policy.max_imu_backlog = 50;
    Packetizer packetizer(policy);
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));
    auto first = packetizer.feed(scan(100));
    REQUIRE(first.has_value());

    // 下游一直不报进度：触发帧全丢，IMU 留着等下一包，超出上限就如实标缺口
    for (std::int64_t i = 2; i <= 20; ++i) {
        for (std::int64_t t = (i - 1) * 100 + 10; t <= i * 100; t += 10) packetizer.feed(imu(t));
        CHECK_FALSE(packetizer.feed(scan(i * 100)).has_value());
        CHECK(packetizer.imuBacklog() <= 50);
    }
    CHECK(packetizer.droppedTriggers() == 19);
    CHECK(packetizer.imuBacklogDrops() > 0);

    packetizer.notifyConsumedThrough(first->sequence);
    for (std::int64_t t = 2010; t <= 2100; t += 10) packetizer.feed(imu(t));
    auto next = packetizer.feed(scan(2100));
    REQUIRE(next.has_value());
    CHECK(next->dropped_triggers == 19);
    CHECK(next->gap == GapReason::kImuGap);  // 这回是真的缺了一段
    CHECK_FALSE(next->supportComplete());
}

TEST_CASE("不要 IMU 时同样按在途上限丢帧", "[framework][packetizer]") {
    PacketizerPolicy policy;
    policy.imu = ImuRequirement::kNotRequired;
    policy.max_in_flight = 2;
    Packetizer packetizer(policy);
    REQUIRE(packetizer.feed(scan(100)).has_value());
    REQUIRE(packetizer.feed(scan(200)).has_value());
    CHECK_FALSE(packetizer.feed(scan(300)).has_value());
    CHECK(packetizer.droppedTriggers() == 1);
    packetizer.notifyConsumedThrough(1);
    auto next = packetizer.feed(scan(400));
    REQUIRE(next.has_value());
    CHECK(next->dropped_triggers == 1);
    CHECK(next->gap == GapReason::kNone);
}

TEST_CASE("来源结束时不按在途上限丢帧：剩下的帧有限，也没有更新的帧要让路", "[framework][packetizer]") {
    PacketizerPolicy policy;
    policy.max_in_flight = 1;
    Packetizer packetizer(policy);
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));
    REQUIRE(packetizer.feed(scan(100)).has_value());

    // IMU 只到 150，下面两帧都还在等括点，来源就结束了
    for (std::int64_t t = 110; t <= 150; t += 10) packetizer.feed(imu(t));
    CHECK_FALSE(packetizer.feed(scan(200)).has_value());
    CHECK_FALSE(packetizer.feed(scan(300)).has_value());
    CHECK(packetizer.droppedTriggers() == 0);

    REQUIRE(packetizer.flush().has_value());
    REQUIRE(packetizer.flush().has_value());
    CHECK_FALSE(packetizer.flush().has_value());
    CHECK(packetizer.droppedTriggers() == 0);
    CHECK(packetizer.inFlight() == 3);
}

TEST_CASE("等括点的触发帧太多时丢最旧的：IMU 同样并入下一包", "[framework][packetizer]") {
    PacketizerPolicy policy;
    policy.max_pending_triggers = 2;
    Packetizer packetizer(policy);
    for (std::int64_t t = 0; t <= 100; t += 10) packetizer.feed(imu(t));
    REQUIRE(packetizer.feed(scan(100)).has_value());

    // 三帧都在等括点，容量 2：scan(200) 被挤掉
    CHECK_FALSE(packetizer.feed(scan(200)).has_value());
    CHECK_FALSE(packetizer.feed(scan(300)).has_value());
    CHECK_FALSE(packetizer.feed(scan(400)).has_value());
    CHECK(packetizer.droppedTriggers() == 1);
    CHECK(packetizer.pendingTriggers() == 2);

    for (std::int64_t t = 110; t <= 300; t += 10) packetizer.ingest(imu(t));
    auto next = packetizer.poll();
    REQUIRE(next.has_value());
    CHECK(next->reference_time == Timestamp{300});
    CHECK(next->dropped_triggers == 1);
    CHECK(next->gap == GapReason::kNone);
    CHECK(next->propagation.begin == Timestamp{100});
    CHECK(next->supportComplete());
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
