// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#include <SimpleSLAM/record/format.hpp>

#include <array>

namespace simpleslam::record {

namespace {

std::array<std::uint32_t, 256> makeCrcTable() {
    std::array<std::uint32_t, 256> table{};
    for (std::uint32_t i = 0; i < 256; ++i) {
        std::uint32_t value = i;
        for (int bit = 0; bit < 8; ++bit) {
            value = (value & 1U) != 0U ? (0xEDB88320U ^ (value >> 1)) : (value >> 1);
        }
        table[i] = value;
    }
    return table;
}

const std::array<std::uint32_t, 256>& crcTable() {
    static const std::array<std::uint32_t, 256> table = makeCrcTable();
    return table;
}

}  // namespace

std::uint32_t crc32(const std::uint8_t* data, std::size_t size) noexcept {
    const auto& table = crcTable();
    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t i = 0; i < size; ++i) {
        crc = table[(crc ^ data[i]) & 0xFFU] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFU;
}

}  // namespace simpleslam::record
