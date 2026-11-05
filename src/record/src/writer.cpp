// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/record/writer.hpp>

#include <sstream>

namespace simpleslam::record {

namespace {

/// 文件头是逐行的 `键: 值`，所以值里的换行必须转义，否则一个带换行的名字
/// 就能凭空造出字段。转义规则只有三条，反向解析见 reader.cpp 的 unescapeField()。
std::string escapeField(const std::string& value) {
    std::string out;
    out.reserve(value.size());
    for (const char c : value) {
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            default: out.push_back(c); break;
        }
    }
    return out;
}

std::string encodeMetadata(const RecordMetadata& metadata) {
    // 手写而不是用 YAML 库：文件头要能用 head -c 看懂，也不该为了它多一个依赖。
    std::ostringstream out;
    out << "name: " << escapeField(metadata.name) << '\n'
        << "created_at: " << escapeField(metadata.created_at) << '\n'
        << "config_hash: " << escapeField(metadata.config_hash) << '\n'
        << "code_revision: " << escapeField(metadata.code_revision) << '\n'
        << "session: " << metadata.session << '\n'
        << "seed: " << metadata.seed << '\n'
        << "note: " << escapeField(metadata.note) << '\n';
    return out.str();
}

void encodePointCloud(ByteWriter& out, const fw::PointCloud& points) {
    const std::uint32_t count = static_cast<std::uint32_t>(points.size());
    out.u32(count);
    // 可选列用位标志表达"有没有"，而不是用长度为零冒充——空列与全零列不是一回事
    std::uint8_t columns = 0;
    if (!points.intensity.empty()) columns |= 0x1U;
    if (!points.time_offset_ns.empty()) columns |= 0x2U;
    if (!points.ring.empty()) columns |= 0x4U;
    out.u8(columns);
    for (std::uint32_t i = 0; i < count; ++i) out.f32(points.x[i]);
    for (std::uint32_t i = 0; i < count; ++i) out.f32(points.y[i]);
    for (std::uint32_t i = 0; i < count; ++i) out.f32(points.z[i]);
    if ((columns & 0x1U) != 0U) {
        for (std::uint32_t i = 0; i < count; ++i) out.f32(points.intensity[i]);
    }
    if ((columns & 0x2U) != 0U) {
        for (std::uint32_t i = 0; i < count; ++i) out.i32(points.time_offset_ns[i]);
    }
    if ((columns & 0x4U) != 0U) {
        for (std::uint32_t i = 0; i < count; ++i) out.u16(points.ring[i]);
    }
}

}  // namespace

void RecordWriter::open(const std::string& path, const RecordMetadata& metadata) {
    close();
    file_.open(path, std::ios::binary | std::ios::trunc);
    if (!file_) throw RecordError("打不开录制文件: " + path);
    path_ = path;

    const std::string meta = encodeMetadata(metadata);
    ByteWriter header;
    for (const char c : kMagic) header.u8(static_cast<std::uint8_t>(c));
    header.u16(kFormatVersion);
    header.u64(0);  // 标志位，暂无
    header.u32(static_cast<std::uint32_t>(meta.size()));
    file_.write(reinterpret_cast<const char*>(header.bytes().data()),
                static_cast<std::streamsize>(header.size()));
    file_.write(meta.data(), static_cast<std::streamsize>(meta.size()));
    if (!file_) throw RecordError("写录制文件头失败: " + path);
    bytes_ = header.size() + meta.size();
    records_ = 0;
}

void RecordWriter::close() {
    if (!file_.is_open()) return;
    // 正常收尾：写下条数。没有它，"恰好删掉最后一条"与"正常写完"无法区分。
    ByteWriter payload;
    payload.u64(records_);
    const std::string path = path_;
    try {
        emit(RecordKind::kEndOfStream, 0, payload);
        file_.flush();
    } catch (...) {
        file_.close();
        throw;
    }
    // 落盘失败在流上只是一个 badbit，不检查就会把半个文件当成录完了；
    // close() 自己也可能失败（缓冲区里最后一段写不进去），关完还要再查一次
    bool failed = !file_.good();
    file_.close();
    failed = failed || file_.fail();
    if (failed) throw RecordError("录制收尾写入失败（磁盘满或超出尺寸上限）: " + path);
}

void RecordWriter::flush() {
    if (!file_.is_open()) return;
    file_.flush();
    if (!file_.good()) throw RecordError("录制落盘失败: " + path_);
}

void RecordWriter::emit(RecordKind kind, std::int64_t time_ns, const ByteWriter& payload) {
    if (!file_.is_open()) throw RecordError("录制文件未打开");
    ByteWriter header;
    header.u32(static_cast<std::uint32_t>(kind));
    header.u32(static_cast<std::uint32_t>(payload.size()));
    header.i64(time_ns);

    // 校验和覆盖"头 + 载荷"：解释载荷要靠头里的种类与长度，它们不能不设防
    std::vector<std::uint8_t> checked;
    checked.reserve(header.size() + payload.size());
    checked.insert(checked.end(), header.bytes().begin(), header.bytes().end());
    checked.insert(checked.end(), payload.bytes().begin(), payload.bytes().end());
    ByteWriter trailer;
    trailer.u32(crc32(checked.data(), checked.size()));

    file_.write(reinterpret_cast<const char*>(header.bytes().data()),
                static_cast<std::streamsize>(header.size()));
    file_.write(reinterpret_cast<const char*>(payload.bytes().data()),
                static_cast<std::streamsize>(payload.size()));
    file_.write(reinterpret_cast<const char*>(trailer.bytes().data()),
                static_cast<std::streamsize>(trailer.size()));
    if (!file_) throw RecordError("写录制记录失败: " + path_);
    if (kind != RecordKind::kEndOfStream) ++records_;
    bytes_ += header.size() + payload.size() + trailer.size();
}

void RecordWriter::write(const fw::ImuSample& sample) {
    scratch_.clear();
    scratch_.u32(sample.domain.value);
    for (const double v : sample.gyro) scratch_.f64(v);
    for (const double v : sample.accel) scratch_.f64(v);
    emit(RecordKind::kImuSample, sample.time.count(), scratch_);
}

void RecordWriter::write(const fw::LidarScan& scan) {
    scratch_.clear();
    scratch_.u32(scan.domain.value);
    scratch_.i64(scan.coverage.begin.count());
    scratch_.i64(scan.coverage.end.count());
    scratch_.text(scan.sensor);
    scratch_.u8(static_cast<std::uint8_t>(scan.motion_compensation));
    encodePointCloud(scratch_, scan.points);
    emit(RecordKind::kLidarScan, scan.reference_time.count(), scratch_);
}

void RecordWriter::write(const fw::CameraFrame& frame) {
    scratch_.clear();
    scratch_.u32(frame.domain.value);
    scratch_.i64(frame.exposure.begin.count());
    scratch_.i64(frame.exposure.end.count());
    scratch_.u8(static_cast<std::uint8_t>(frame.shutter));
    scratch_.i64(frame.line_time.count());
    scratch_.u32(frame.width);
    scratch_.u32(frame.height);
    scratch_.u8(frame.channels);
    scratch_.text(frame.sensor);
    scratch_.blob(frame.data.data(), frame.data.size());
    emit(RecordKind::kCameraFrame, frame.capture_time.count(), scratch_);
}

void RecordWriter::write(const fw::GnssFix& fix) {
    scratch_.clear();
    scratch_.u32(fix.domain.value);
    for (const double v : fix.ecef) scratch_.f64(v);
    for (const double v : fix.covariance) scratch_.f64(v);
    scratch_.u8(static_cast<std::uint8_t>(fix.kind));
    scratch_.u8(fix.satellites);
    emit(RecordKind::kGnssFix, fix.time.count(), scratch_);
}

std::size_t RecordWriter::write(const fw::SensorDatum& datum) {
    const std::uint64_t before = bytes_;
    std::visit(
        [this](const auto& ptr) {
            if (ptr) write(*ptr);
        },
        datum);
    return static_cast<std::size_t>(bytes_ - before);
}

}  // namespace simpleslam::record
