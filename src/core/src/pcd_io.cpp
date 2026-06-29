#include <SimpleSLAM/core/math/pcd_io.hpp>

#include <cassert>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace simpleslam::pcd_io {

void writePCD(const LidarScan& scan, const std::string& path) {
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("pcd_io: cannot open for writing: " + path);
    }

    const bool has_intensity = scan.hasIntensities();
    const bool has_normal = scan.hasNormals();

    // 构建字段描述
    std::string fields = "x y z";
    std::string sizes = "4 4 4";
    std::string types = "F F F";
    std::string counts = "1 1 1";
    int point_size = 12;

    if (has_intensity) {
        fields += " intensity";
        sizes += " 4";
        types += " F";
        counts += " 1";
        point_size += 4;
    }
    if (has_normal) {
        fields += " normal_x normal_y normal_z";
        sizes += " 4 4 4";
        types += " F F F";
        counts += " 1 1 1";
        point_size += 12;
    }

    // 写入 PCD 头
    file << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\n"
         << "FIELDS " << fields << "\n"
         << "SIZE " << sizes << "\n"
         << "TYPE " << types << "\n"
         << "COUNT " << counts << "\n"
         << "WIDTH " << scan.size() << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << scan.size() << "\n"
         << "DATA binary\n";

    // 写入二进制数据
    for (size_t i = 0; i < scan.size(); ++i) {
        const auto& p = scan.points[i];
        file.write(reinterpret_cast<const char*>(p.data()), 12);
        if (has_intensity) {
            float intensity = (*scan.intensities)[i];
            file.write(reinterpret_cast<const char*>(&intensity), 4);
        }
        if (has_normal) {
            const auto& n = (*scan.normals)[i];
            file.write(reinterpret_cast<const char*>(n.data()), 12);
        }
    }
}

LidarScan readPCD(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("pcd_io: cannot open for reading: " + path);
    }

    // 解析头部
    size_t num_points = 0;
    std::vector<std::string> field_names;
    std::string line;
    bool data_found = false;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        iss >> key;

        if (key == "FIELDS") {
            std::string field;
            while (iss >> field) {
                field_names.push_back(field);
            }
        } else if (key == "POINTS") {
            iss >> num_points;
        } else if (key == "DATA") {
            std::string data_type;
            iss >> data_type;
            if (data_type != "binary") {
                throw std::runtime_error("pcd_io: only binary DATA format supported");
            }
            data_found = true;
            break;
        }
    }

    if (!data_found || num_points == 0) {
        throw std::runtime_error("pcd_io: invalid PCD header in " + path);
    }

    // 确定字段布局
    bool has_intensity = false;
    bool has_normal = false;
    int intensity_offset = -1;
    int normal_offset = -1;
    int field_offset = 0;

    for (const auto& name : field_names) {
        if (name == "intensity") {
            has_intensity = true;
            intensity_offset = field_offset;
        } else if (name == "normal_x") {
            has_normal = true;
            normal_offset = field_offset;
        }
        field_offset += 4;  // 每个字段 4 字节（float）
    }

    const int point_stride = field_offset;

    // 读取二进制数据
    LidarScan scan;
    scan.points.resize(num_points);
    if (has_intensity) scan.intensities.emplace().resize(num_points);
    if (has_normal) scan.normals.emplace().resize(num_points);

    std::vector<char> buffer(static_cast<size_t>(point_stride));
    for (size_t i = 0; i < num_points; ++i) {
        file.read(buffer.data(), point_stride);
        if (!file) {
            throw std::runtime_error("pcd_io: premature end of data in " + path);
        }

        std::memcpy(scan.points[i].data(), buffer.data(), 12);

        if (has_intensity && intensity_offset >= 0) {
            std::memcpy(&(*scan.intensities)[i],
                        buffer.data() + intensity_offset, 4);
        }
        if (has_normal && normal_offset >= 0) {
            std::memcpy((*scan.normals)[i].data(),
                        buffer.data() + normal_offset, 12);
        }
    }

    assert(scan.isConsistent());
    return scan;
}

}  // namespace simpleslam::pcd_io
