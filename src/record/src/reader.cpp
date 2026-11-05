// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/record/reader.hpp>

#include <sstream>

namespace simpleslam::record {

namespace {

/// 载荷长度的合理上限（256 MiB）。头被篡改时不能照着它去分配内存。
/// 头里的长度字段来自文件，一律先设上限再据它分配——损坏的文件不该让读取器
/// 先申请 4 GiB 再失败。
constexpr std::uint32_t kMaxPayloadBytes = 256U * 1024U * 1024U;
constexpr std::uint32_t kMaxMetadataBytes = 1U * 1024U * 1024U;

/// 文件头逐行 `键: 值`，值里的换行在写入时已转义，这里还原。见 writer.cpp 的
/// escapeField()：只有 `\\`、`\n`、`\r` 三条规则，未知转义原样保留。
std::string unescapeField(const std::string& value) {
    std::string out;
    out.reserve(value.size());
    for (std::size_t i = 0; i < value.size(); ++i) {
        if (value[i] != '\\' || i + 1 >= value.size()) {
            out.push_back(value[i]);
            continue;
        }
        switch (value[i + 1]) {
            case '\\': out.push_back('\\'); ++i; break;
            case 'n': out.push_back('\n'); ++i; break;
            case 'r': out.push_back('\r'); ++i; break;
            default: out.push_back(value[i]); break;
        }
    }
    return out;
}

/// 解码文件头。version 决定字符串怎么解释：版本 3 起按转义规则还原，
/// 版本 2 的反斜杠就是反斜杠——同一段字节在两个版本下含义不同，不能一律按新规则读。
RecordMetadata decodeMetadata(const std::string& text, std::uint16_t version) {
    RecordMetadata metadata;
    std::istringstream input(text);
    std::string line;
    while (std::getline(input, line)) {
        const auto colon = line.find(": ");
        const auto key = colon == std::string::npos ? line.substr(0, line.find(':'))
                                                    : line.substr(0, colon);
        const std::string raw = colon == std::string::npos ? std::string{} : line.substr(colon + 2);
        const std::string value = version >= 3 ? unescapeField(raw) : raw;
        if (key == "name") metadata.name = value;
        else if (key == "created_at") metadata.created_at = value;
        else if (key == "config_hash") metadata.config_hash = value;
        else if (key == "code_revision") metadata.code_revision = value;
        else if (key == "note") metadata.note = value;
        else if (key == "session" && !value.empty()) metadata.session = std::stoull(value);
        else if (key == "seed" && !value.empty()) metadata.seed = std::stoll(value);
    }
    return metadata;
}

/// 已知字段解码完之后，载荷必须恰好用完。
///
/// 校验和只证明字节没坏，不证明语义解码完整：多出来的字节可能是更新版本的字段、
/// 也可能是写坏了的数据，两种情况都不能当作"读完了"。版本规则里没有定义扩展区，
/// 所以一律拒绝。
void requireConsumed(const ByteReader& in, const char* what) {
    if (in.remaining() != 0) {
        throw RecordError(std::string(what) + "记录的载荷有 " + std::to_string(in.remaining())
                          + " 字节未解码的尾随数据");
    }
}

fw::PointCloud decodePointCloud(ByteReader& in) {
    fw::PointCloud points;
    const std::uint32_t count = in.u32();
    const std::uint8_t columns = in.u8();
    if ((columns & ~0x7U) != 0U) throw RecordError("点云记录含未知列标志");
    // 先核对"声明的点数与剩余字节相符"，再分配。反过来的话，一个损坏的点数
    // 会在越界报错之前先吃掉几 GiB 内存。
    std::size_t per_point = 3 * sizeof(float);
    if ((columns & 0x1U) != 0U) per_point += sizeof(float);
    if ((columns & 0x2U) != 0U) per_point += sizeof(std::int32_t);
    if ((columns & 0x4U) != 0U) per_point += sizeof(std::uint16_t);
    if (static_cast<std::size_t>(count) * per_point > in.remaining()) {
        throw RecordError("点云记录声明的点数与载荷长度不符: " + std::to_string(count));
    }
    points.x.resize(count);
    points.y.resize(count);
    points.z.resize(count);
    for (std::uint32_t i = 0; i < count; ++i) points.x[i] = in.f32();
    for (std::uint32_t i = 0; i < count; ++i) points.y[i] = in.f32();
    for (std::uint32_t i = 0; i < count; ++i) points.z[i] = in.f32();
    if ((columns & 0x1U) != 0U) {
        points.intensity.resize(count);
        for (std::uint32_t i = 0; i < count; ++i) points.intensity[i] = in.f32();
    }
    if ((columns & 0x2U) != 0U) {
        points.time_offset_ns.resize(count);
        for (std::uint32_t i = 0; i < count; ++i) points.time_offset_ns[i] = in.i32();
    }
    if ((columns & 0x4U) != 0U) {
        points.ring.resize(count);
        for (std::uint32_t i = 0; i < count; ++i) points.ring[i] = in.u16();
    }
    if (!points.consistent()) throw RecordError("解码出的点云各列长度不一致");
    return points;
}

}  // namespace

void RecordReader::open(const std::string& path) {
    file_.open(path, std::ios::binary);
    if (!file_) throw RecordError("打不开录制文件: " + path);
    path_ = path;
    truncated_ = false;
    complete_ = false;
    records_ = 0;

    std::array<char, sizeof(kMagic) + 2 + 8 + 4> header{};
    file_.read(header.data(), static_cast<std::streamsize>(header.size()));
    if (file_.gcount() != static_cast<std::streamsize>(header.size())) {
        throw RecordError("录制文件头不完整: " + path);
    }
    for (std::size_t i = 0; i < sizeof(kMagic); ++i) {
        if (header[i] != kMagic[i]) throw RecordError("不是 SimpleSLAM 录制文件: " + path);
    }
    ByteReader reader(reinterpret_cast<const std::uint8_t*>(header.data()), header.size());
    for (std::size_t i = 0; i < sizeof(kMagic); ++i) (void)reader.u8();
    const std::uint16_t version = reader.u16();
    if (version < kOldestReadableVersion || version > kFormatVersion) {
        throw RecordError("录制格式版本不支持: " + std::to_string(version) + "（可读 "
                          + std::to_string(kOldestReadableVersion) + " 到 "
                          + std::to_string(kFormatVersion) + "）");
    }
    version_ = version;
    (void)reader.u64();
    const std::uint32_t meta_length = reader.u32();
    // 头里的长度来自文件，不可信：先设上限再按它分配
    if (meta_length > kMaxMetadataBytes) {
        throw RecordError("录制文件头声明的元数据长度不合理: " + std::to_string(meta_length));
    }

    raw_metadata_.resize(meta_length);
    if (meta_length > 0) {
        file_.read(raw_metadata_.data(), static_cast<std::streamsize>(meta_length));
        if (file_.gcount() != static_cast<std::streamsize>(meta_length)) {
            throw RecordError("录制文件的元数据不完整: " + path);
        }
    }
    metadata_ = decodeMetadata(raw_metadata_, version_);
}

std::optional<fw::SensorDatum> RecordReader::next() {
    if (!file_.is_open() || truncated_) return std::nullopt;

    std::array<char, 16> header{};
    file_.read(header.data(), static_cast<std::streamsize>(header.size()));
    const auto got = file_.gcount();
    if (got == 0) return std::nullopt;  // 正常结束
    if (got != static_cast<std::streamsize>(header.size())) {
        truncated_ = true;
        return std::nullopt;
    }

    ByteReader head(reinterpret_cast<const std::uint8_t*>(header.data()), header.size());
    const auto kind = static_cast<RecordKind>(head.u32());
    const std::uint32_t length = head.u32();
    const std::int64_t time_ns = head.i64();
    if (length > kMaxPayloadBytes) {
        throw RecordError("录制记录声明的载荷长度不合理: " + std::to_string(length));
    }

    payload_.resize(length);
    if (length > 0) {
        file_.read(reinterpret_cast<char*>(payload_.data()), static_cast<std::streamsize>(length));
        if (file_.gcount() != static_cast<std::streamsize>(length)) {
            truncated_ = true;
            return std::nullopt;
        }
    }
    std::array<char, 4> trailer{};
    file_.read(trailer.data(), 4);
    if (file_.gcount() != 4) {
        truncated_ = true;
        return std::nullopt;
    }
    ByteReader tail(reinterpret_cast<const std::uint8_t*>(trailer.data()), 4);
    const std::uint32_t expected = tail.u32();
    std::vector<std::uint8_t> checked;
    checked.reserve(header.size() + payload_.size());
    checked.insert(checked.end(), header.begin(), header.end());
    checked.insert(checked.end(), payload_.begin(), payload_.end());
    if (crc32(checked.data(), checked.size()) != expected) {
        throw RecordError("录制记录校验和不符（第 " + std::to_string(records_ + 1) + " 条）");
    }

    if (kind == RecordKind::kEndOfStream) {
        ByteReader footer(payload_.data(), payload_.size());
        const std::uint64_t declared = footer.u64();
        requireConsumed(footer, "收尾");
        if (declared != records_) {
            throw RecordError("录制收尾声明 " + std::to_string(declared) + " 条，实读 "
                              + std::to_string(records_) + " 条");
        }
        // 收尾之后不该再有字节。有的话这份文件是被拼接或改写过的，不能算完整。
        if (file_.peek() != std::char_traits<char>::eof()) {
            throw RecordError("录制收尾记录之后还有多余字节");
        }
        complete_ = true;
        return std::nullopt;  // 正常结束
    }

    ByteReader in(payload_.data(), payload_.size());
    ++records_;
    switch (kind) {
        case RecordKind::kEndOfStream: break;  // 上面已处理并返回
        case RecordKind::kImuSample: {
            auto sample = std::make_shared<fw::ImuSample>();
            sample->domain = fw::ClockDomainId{in.u32()};
            sample->time = fw::Timestamp{time_ns};
            for (auto& v : sample->gyro) v = in.f64();
            for (auto& v : sample->accel) v = in.f64();
            requireConsumed(in, "IMU");
            return fw::SensorDatum{fw::Ptr<fw::ImuSample>(sample)};
        }
        case RecordKind::kLidarScan: {
            auto scan = std::make_shared<fw::LidarScan>();
            scan->domain = fw::ClockDomainId{in.u32()};
            scan->reference_time = fw::Timestamp{time_ns};
            scan->coverage.begin = fw::Timestamp{in.i64()};
            scan->coverage.end = fw::Timestamp{in.i64()};
            scan->sensor = in.text();
            if (version_ >= 3) {
                const std::uint8_t compensation = in.u8();
                if (compensation > static_cast<std::uint8_t>(fw::MotionCompensation::kCompensated)) {
                    throw RecordError("激光记录的运动补偿状态取值未知: "
                                      + std::to_string(compensation));
                }
                scan->motion_compensation = static_cast<fw::MotionCompensation>(compensation);
            }
            scan->points = decodePointCloud(in);
            requireConsumed(in, "激光");
            return fw::SensorDatum{fw::Ptr<fw::LidarScan>(scan)};
        }
        case RecordKind::kCameraFrame: {
            auto frame = std::make_shared<fw::CameraFrame>();
            frame->domain = fw::ClockDomainId{in.u32()};
            frame->capture_time = fw::Timestamp{time_ns};
            frame->exposure.begin = fw::Timestamp{in.i64()};
            frame->exposure.end = fw::Timestamp{in.i64()};
            const std::uint8_t shutter = in.u8();
            if (shutter > static_cast<std::uint8_t>(fw::ShutterKind::kRolling)) {
                throw RecordError("图像记录的快门类型取值未知: " + std::to_string(shutter));
            }
            frame->shutter = static_cast<fw::ShutterKind>(shutter);
            frame->line_time = fw::Duration{in.i64()};
            frame->width = in.u32();
            frame->height = in.u32();
            frame->channels = in.u8();
            frame->sensor = in.text();
            frame->data = in.blob();
            if (!frame->consistent()) throw RecordError("解码出的图像尺寸与数据长度不符");
            requireConsumed(in, "图像");
            return fw::SensorDatum{fw::Ptr<fw::CameraFrame>(frame)};
        }
        case RecordKind::kGnssFix: {
            auto fix = std::make_shared<fw::GnssFix>();
            fix->domain = fw::ClockDomainId{in.u32()};
            fix->time = fw::Timestamp{time_ns};
            for (auto& v : fix->ecef) v = in.f64();
            for (auto& v : fix->covariance) v = in.f64();
            const std::uint8_t kind_value = in.u8();
            if (kind_value > static_cast<std::uint8_t>(fw::GnssFixKind::kFixedRtk)) {
                throw RecordError("GNSS 记录的定位类型取值未知: " + std::to_string(kind_value));
            }
            fix->kind = static_cast<fw::GnssFixKind>(kind_value);
            fix->satellites = in.u8();
            requireConsumed(in, "GNSS");
            return fw::SensorDatum{fw::Ptr<fw::GnssFix>(fix)};
        }
    }
    throw RecordError("未知的记录种类: " + std::to_string(static_cast<std::uint32_t>(kind)));
}

}  // namespace simpleslam::record
