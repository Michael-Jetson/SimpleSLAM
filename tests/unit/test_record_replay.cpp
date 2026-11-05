// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 录制与回放：编码往返逐位一致、"完整"与"完好的前缀"可分、
/// 校验和覆盖记录头、实验回放包钉住配置与输入。

#include <SimpleSLAM/record/bundle.hpp>
#include <SimpleSLAM/record/diagnostic_file_sink.hpp>
#include <SimpleSLAM/record/reader.hpp>
#include <SimpleSLAM/record/writer.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cstring>
#include <filesystem>
#include <fstream>

using namespace simpleslam;
using namespace simpleslam::record;
using Catch::Approx;

namespace {

std::filesystem::path tempFile(const char* name) {
    return std::filesystem::temp_directory_path() / name;
}

fw::ImuSample makeImu(std::int64_t ns) {
    fw::ImuSample sample;
    sample.domain = fw::ClockDomainId{3};
    sample.time = fw::Timestamp{ns};
    sample.gyro = {0.1, -0.2, 0.3};
    sample.accel = {0.0, 0.0, 9.81};
    return sample;
}

fw::LidarScan makeScan(std::int64_t ns, std::size_t points) {
    fw::LidarScan scan;
    scan.domain = fw::ClockDomainId{1};
    scan.reference_time = fw::Timestamp{ns};
    scan.coverage = fw::TimeRange{fw::Timestamp{ns - 100'000'000}, fw::Timestamp{ns}};
    scan.sensor = "ouster-0";
    for (std::size_t i = 0; i < points; ++i) {
        scan.points.push(static_cast<float>(i) * 0.5F, 1.25F, -3.5F);
        scan.points.intensity.push_back(static_cast<float>(i));
        scan.points.time_offset_ns.push_back(static_cast<std::int32_t>(i) * 1000);
        scan.points.ring.push_back(static_cast<std::uint16_t>(i % 64));
    }
    return scan;
}

fw::GnssFix makeGnss(std::int64_t ns) {
    fw::GnssFix fix;
    fix.time = fw::Timestamp{ns};
    fix.ecef = {-2694045.0, 4293642.0, 3857878.0};
    fix.covariance.fill(0.25);
    fix.kind = fw::GnssFixKind::kFixedRtk;
    fix.satellites = 21;
    return fix;
}

/// 小型来源：把给定的数据按顺序吐出来。
class VectorSource final : public fw::ISensorSource {
public:
    explicit VectorSource(std::vector<fw::SensorDatum> data) : data_(std::move(data)) {}
    std::optional<fw::SensorDatum> next() override {
        if (cursor_ >= data_.size()) return std::nullopt;
        return data_[cursor_++];
    }

private:
    std::vector<fw::SensorDatum> data_;
    std::size_t cursor_ = 0;
};

}  // namespace

TEST_CASE("四种数据编码往返逐位一致", "[record]") {
    const auto path = tempFile("simpleslam_record_roundtrip.ssrec");
    RecordMetadata metadata;
    metadata.name = "roundtrip";
    metadata.config_hash = "cafebabe12345678";
    metadata.session = 7;
    metadata.seed = -42;

    {
        RecordWriter writer(path.string(), metadata);
        writer.write(makeImu(1000));
        writer.write(makeScan(2000, 5));
        fw::CameraFrame frame;
        frame.capture_time = fw::Timestamp{3000};
        frame.exposure = fw::TimeRange{fw::Timestamp{2900}, fw::Timestamp{3100}};
        frame.shutter = fw::ShutterKind::kRolling;
        frame.line_time = fw::Duration{50};
        frame.width = 2;
        frame.height = 2;
        frame.channels = 1;
        frame.sensor = "cam0";
        frame.data = {1, 2, 3, 4};
        writer.write(frame);
        writer.write(makeGnss(4000));
        CHECK(writer.recordCount() == 4);
    }

    RecordReader reader(path.string());
    CHECK(reader.metadata().name == "roundtrip");
    CHECK(reader.metadata().config_hash == "cafebabe12345678");
    CHECK(reader.metadata().session == 7);
    CHECK(reader.metadata().seed == -42);

    auto imu = reader.next();
    REQUIRE(imu.has_value());
    const auto& imu_ptr = std::get<fw::Ptr<fw::ImuSample>>(*imu);
    CHECK(imu_ptr->time.count() == 1000);
    CHECK(imu_ptr->domain.value == 3);
    CHECK(imu_ptr->gyro[1] == Approx(-0.2));
    CHECK(imu_ptr->accel[2] == Approx(9.81));

    auto scan = reader.next();
    REQUIRE(scan.has_value());
    const auto& scan_ptr = std::get<fw::Ptr<fw::LidarScan>>(*scan);
    CHECK(scan_ptr->reference_time.count() == 2000);
    CHECK(scan_ptr->sensor == "ouster-0");
    REQUIRE(scan_ptr->points.size() == 5);
    CHECK(scan_ptr->points.x[3] == Approx(1.5F));
    CHECK(scan_ptr->points.time_offset_ns[4] == 4000);
    CHECK(scan_ptr->points.ring[2] == 2);
    CHECK(scan_ptr->coverage.begin.count() == 2000 - 100'000'000);

    auto frame = reader.next();
    REQUIRE(frame.has_value());
    const auto& frame_ptr = std::get<fw::Ptr<fw::CameraFrame>>(*frame);
    CHECK(frame_ptr->shutter == fw::ShutterKind::kRolling);
    CHECK(frame_ptr->line_time.count() == 50);
    CHECK(frame_ptr->data == std::vector<std::uint8_t>{1, 2, 3, 4});

    auto gnss = reader.next();
    REQUIRE(gnss.has_value());
    const auto& gnss_ptr = std::get<fw::Ptr<fw::GnssFix>>(*gnss);
    CHECK(gnss_ptr->kind == fw::GnssFixKind::kFixedRtk);
    CHECK(gnss_ptr->satellites == 21);

    CHECK_FALSE(reader.next().has_value());
    CHECK_FALSE(reader.truncated());
    CHECK(reader.complete());  // 读到了收尾记录，条数相符
    std::filesystem::remove(path);
}

namespace {
/// 一条 IMU 记录 = 16 头 + 52 载荷 + 4 校验；收尾记录 = 16 + 8 + 4。
constexpr std::streamoff kImuRecordBytes = 16 + 52 + 4;
constexpr std::streamoff kFooterBytes = 16 + 8 + 4;

/// 文件头长度 = 20 + 元数据长度。改字节要先知道记录从哪开始。
std::streamoff headerBytes(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    char head[20] = {};
    file.read(head, 20);
    std::uint32_t meta_len = 0;
    std::memcpy(&meta_len, head + 16, 4);
    return 20 + static_cast<std::streamoff>(meta_len);
}
}  // namespace

TEST_CASE("可选列为空与全零不是一回事", "[record]") {
    const auto path = tempFile("simpleslam_record_columns.ssrec");
    {
        RecordWriter writer(path.string());
        fw::LidarScan scan;
        scan.reference_time = fw::Timestamp{10};
        scan.points.push(1.0F, 2.0F, 3.0F);  // 只有 xyz，其余列不存在
        writer.write(scan);
    }
    RecordReader reader(path.string());
    auto datum = reader.next();
    REQUIRE(datum.has_value());
    const auto& scan = std::get<fw::Ptr<fw::LidarScan>>(*datum);
    CHECK(scan->points.size() == 1);
    CHECK(scan->points.intensity.empty());
    CHECK(scan->points.ring.empty());
    std::filesystem::remove(path);
}

TEST_CASE("半条记录被截断时认得出来", "[record]") {
    const auto path = tempFile("simpleslam_record_truncated.ssrec");
    {
        RecordWriter writer(path.string());
        for (int i = 0; i < 5; ++i) writer.write(makeImu(i * 1000));
    }
    // 模拟录制进程被杀：砍掉收尾记录和最后一条的一部分
    const auto size = static_cast<std::streamoff>(std::filesystem::file_size(path));
    std::filesystem::resize_file(path, static_cast<std::uintmax_t>(size - kFooterBytes - 7));

    RecordReader reader(path.string());
    std::size_t count = 0;
    while (reader.next()) ++count;
    CHECK(count == 4);           // 前四条完好
    CHECK(reader.truncated());   // 半条记录的残骸被认出来了
    CHECK_FALSE(reader.complete());
    std::filesystem::remove(path);
}

TEST_CASE("恰好删掉最后一条完整记录也认得出来", "[record]") {
    // 没有结束标记时，这种截断与正常写完一模一样。
    const auto path = tempFile("simpleslam_record_clean_cut.ssrec");
    {
        RecordWriter writer(path.string());
        for (int i = 0; i < 5; ++i) writer.write(makeImu(i * 1000));
    }
    const auto size = static_cast<std::streamoff>(std::filesystem::file_size(path));
    std::filesystem::resize_file(
        path, static_cast<std::uintmax_t>(size - kFooterBytes - kImuRecordBytes));

    RecordReader reader(path.string());
    std::size_t count = 0;
    while (reader.next()) ++count;
    CHECK(count == 4);
    CHECK_FALSE(reader.truncated());  // 边界干净，没有残骸
    CHECK_FALSE(reader.complete());   // 但没有收尾记录 → 不能声称完整
    std::filesystem::remove(path);
}

TEST_CASE("篡改记录头也被校验和抓住", "[record]") {
    // 只校验载荷时，改掉时间戳不会被发现——
    // 而解释载荷正要靠头里的种类与长度。
    const auto path = tempFile("simpleslam_record_header.ssrec");
    {
        RecordWriter writer(path.string());
        for (int i = 0; i < 3; ++i) writer.write(makeImu(100 + i));
    }
    {
        std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
        file.seekp(headerBytes(path) + 8);  // kind(4) + len(4) → 时刻
        const std::uint64_t tampered = 999;
        file.write(reinterpret_cast<const char*>(&tampered), 8);
    }
    RecordReader reader(path.string());
    CHECK_THROWS_AS(reader.next(), RecordError);
    std::filesystem::remove(path);
}

TEST_CASE("载荷位翻转被校验和抓住", "[record]") {
    const auto path = tempFile("simpleslam_record_corrupt.ssrec");
    {
        RecordWriter writer(path.string());
        writer.write(makeScan(1000, 4));
    }
    {
        std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
        file.seekp(headerBytes(path) + 16 + 8);  // 记录头之后的载荷
        char byte = 0;
        file.read(&byte, 1);
        file.seekp(-1, std::ios::cur);
        byte = static_cast<char>(byte ^ 0x40);
        file.write(&byte, 1);
    }
    RecordReader reader(path.string());
    CHECK_THROWS_AS(reader.next(), RecordError);
    std::filesystem::remove(path);
}

TEST_CASE("录制装饰器不改变来源的行为", "[record]") {
    const auto path = tempFile("simpleslam_record_tee.ssrec");
    std::vector<fw::SensorDatum> data;
    for (int i = 0; i < 3; ++i) {
        data.emplace_back(fw::Ptr<fw::ImuSample>(std::make_shared<fw::ImuSample>(makeImu(i))));
    }
    data.emplace_back(fw::Ptr<fw::LidarScan>(std::make_shared<fw::LidarScan>(makeScan(100, 2))));

    auto writer = std::make_shared<RecordWriter>(path.string());
    RecordingSource source(std::make_unique<VectorSource>(data), writer);

    std::size_t seen = 0;
    while (source.next()) ++seen;
    CHECK(seen == 4);
    writer->close();

    ReplaySource replay(path.string());
    std::size_t replayed = 0;
    while (replay.next()) ++replayed;
    CHECK(replayed == 4);
    CHECK_FALSE(replay.truncated());
    CHECK(replay.complete());
    std::filesystem::remove(path);
}

TEST_CASE("实验回放包钉住配置、输入与种子", "[record][bundle]") {
    const auto data_path = tempFile("simpleslam_bundle_input.bin");
    {
        std::ofstream file(data_path, std::ios::binary);
        file << "hello simpleslam";
    }
    std::uint64_t bytes = 0;
    const auto digest = fileDigest(data_path.string(), &bytes);
    CHECK(bytes == 16);
    CHECK(digest.size() == 16);

    ExperimentBundle bundle;
    bundle.name = "demo";
    bundle.created_at = "2026-09-21T09:00:00Z";
    bundle.code_revision = "abc1234";
    bundle.config_hash = "0011223344556677";
    bundle.seed = 17;
    bundle.session = 2;
    bundle.inputs.push_back(InputDigest{"recording", data_path.string(), digest, bytes});
    bundle.metrics = {{"ate.translation.rmse", 0.31}};

    const auto bundle_path = tempFile("simpleslam_bundle.yaml");
    writeBundle(bundle_path.string(), bundle);
    const auto loaded = readBundle(bundle_path.string());
    CHECK(loaded.config_hash == bundle.config_hash);
    CHECK(loaded.seed == 17);
    REQUIRE(loaded.input("recording") != nullptr);
    CHECK(loaded.input("recording")->digest == digest);
    CHECK(loaded.metrics.at("ate.translation.rmse") == Approx(0.31));

    // 改一个字节，指纹就变；回放包的比对因此能说出"输入内容变了"
    {
        std::ofstream file(data_path, std::ios::binary);
        file << "hello simpleslaM";
    }
    ExperimentBundle after = loaded;
    after.inputs.front().digest = fileDigest(data_path.string());
    const auto differences = diffBundles(loaded, after);
    REQUIRE(differences.size() == 1);
    CHECK(differences.front().find("输入内容变了") != std::string::npos);

    std::filesystem::remove(data_path);
    std::filesystem::remove(bundle_path);
}

TEST_CASE("诊断落盘为 JSONL，控制档立刻 flush", "[record][diagnostics]") {
    const auto path = tempFile("simpleslam_diagnostics.jsonl");
    FileDiagnosticSink sink(path.string());

    fw::DiagnosticRecord control;
    control.header.module = "/slam/odometry";
    control.header.frame_sequence = 12;
    control.header.session = fw::SessionId{3};
    control.level = fw::DiagnosticClass::kControl;
    control.payload = fw::EventRecord{"odometry.correction_applied", "身份兼容"};
    sink.submit(control);

    fw::DiagnosticRecord metric;
    metric.header.module = "/slam/odometry";
    metric.level = fw::DiagnosticClass::kMetric;
    metric.payload = fw::MetricRecord{"odometry.inliers", 128.0, "count"};
    sink.submit(metric);

    sink.close();
    CHECK(sink.written() == 2);
    CHECK(sink.failed() == 0);

    std::ifstream file(path);
    std::string first;
    std::string second;
    std::getline(file, first);
    std::getline(file, second);
    CHECK(first.find("\"level\":\"control\"") != std::string::npos);
    CHECK(first.find("\"data_time\":") != std::string::npos);
    CHECK(first.find("odometry.correction_applied") != std::string::npos);
    CHECK(second.find("\"kind\":\"metric\"") != std::string::npos);
    CHECK(second.find("128") != std::string::npos);
    std::filesystem::remove(path);
}

TEST_CASE("元数据里的换行不能凭空造出字段", "[record][format]") {
    // 文件头是逐行的 `键: 值`，值里的换行必须转义
    const auto path = tempFile("simpleslam_meta_escape.ssrec");
    RecordMetadata metadata;
    metadata.name = "第一行\nconfig_hash: 伪造的哈希";
    metadata.config_hash = "real";
    {
        RecordWriter writer(path.string(), metadata);
        writer.close();
    }
    RecordReader reader(path.string());
    CHECK(reader.metadata().name == metadata.name);
    CHECK(reader.metadata().config_hash == "real");  // 没有被注入的那一行覆盖
    std::filesystem::remove(path);
}

TEST_CASE("回放的终态区分读完与读到一半", "[record][replay]") {
    // 一个空的 optional 说不出"读完了"和"读到一半没了"的区别，
    // 而严格评测两者必须分开
    const auto path = tempFile("simpleslam_outcome.ssrec");
    {
        RecordWriter writer(path.string());
        fw::ImuSample sample;
        sample.time = fw::Timestamp{1000};
        writer.write(sample);
        writer.close();
    }

    {
        ReplaySource source(path.string());
        CHECK(source.outcome() == fw::SourceOutcome::kRunning);
        while (source.next()) {
        }
        CHECK(source.outcome() == fw::SourceOutcome::kCleanEof);
    }

    // 砍掉收尾记录：前面的数据仍然可读，但这次输入不完整
    const auto full = static_cast<std::streamoff>(std::filesystem::file_size(path));
    std::filesystem::resize_file(path, static_cast<std::uintmax_t>(full - kFooterBytes));
    {
        ReplaySource source(path.string());
        while (source.next()) {
        }
        CHECK(source.outcome() == fw::SourceOutcome::kIncomplete);
    }
    std::filesystem::remove(path);
}

TEST_CASE("收尾记录之后不该再有字节", "[record][format]") {
    const auto path = tempFile("simpleslam_trailing.ssrec");
    {
        RecordWriter writer(path.string());
        fw::ImuSample sample;
        sample.time = fw::Timestamp{1000};
        writer.write(sample);
        writer.close();
    }
    {
        std::ofstream extra(path, std::ios::binary | std::ios::app);
        extra << "多出来的字节";
    }
    RecordReader reader(path.string());
    CHECK_THROWS_AS([&] { while (reader.next()) {} }(), RecordError);
    std::filesystem::remove(path);
}
