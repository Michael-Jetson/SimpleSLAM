// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

#pragma once

/// @file line_stream.hpp
/// 把 `<<` 写出的每一行变成一条日志：执行器沿用流式的写法（含 `std::fixed`、
/// `std::setprecision`），输出进日志组件：终端上只有正文，run.log 里每行带等级、时刻与来源。

#include <SimpleSLAM/logging/logger.hpp>

#include <ostream>
#include <streambuf>
#include <string>
#include <utility>

namespace simpleslam::app {

class LineStream final : public std::ostream {
public:
    LineStream(logging::Logger logger, logging::Level level)
        : std::ostream(nullptr), buffer_(std::move(logger), level) {
        rdbuf(&buffer_);
    }
    LineStream(const LineStream&) = delete;
    LineStream& operator=(const LineStream&) = delete;
    ~LineStream() override = default;

private:
    class Buffer final : public std::streambuf {
    public:
        Buffer(logging::Logger logger, logging::Level level)
            : logger_(std::move(logger)), level_(level) {}
        Buffer(const Buffer&) = delete;
        Buffer& operator=(const Buffer&) = delete;
        ~Buffer() override {
            if (!line_.empty()) emit();
        }

    protected:
        int_type overflow(int_type ch) override {
            if (traits_type::eq_int_type(ch, traits_type::eof())) return traits_type::not_eof(ch);
            put(traits_type::to_char_type(ch));
            return ch;
        }
        std::streamsize xsputn(const char* text, std::streamsize count) override {
            for (std::streamsize i = 0; i < count; ++i) put(text[i]);
            return count;
        }

    private:
        void put(char c) {
            if (c == '\n') {
                emit();
            } else {
                line_.push_back(c);
            }
        }
        void emit() {
            switch (level_) {
                case logging::Level::kDebug: logger_.debug("{}", line_); break;
                case logging::Level::kInfo: logger_.info("{}", line_); break;
                case logging::Level::kWarn: logger_.warn("{}", line_); break;
                case logging::Level::kError: logger_.error("{}", line_); break;
            }
            line_.clear();
        }

        logging::Logger logger_;
        logging::Level level_;
        std::string line_;
    };

    Buffer buffer_;
};

}  // namespace simpleslam::app
