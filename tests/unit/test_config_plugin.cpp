// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

// 插件槽与目录：按类型名选实现、严格还原参数、报错带路径与可选项、规范化导出与 schema。

#include <SimpleSLAM/algo/builtins.hpp>
#include <SimpleSLAM/config/plugin.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cstdint>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

using namespace simpleslam;
using config::Catalogs;
using config::ConfigError;
using config::PluginImplementation;
using config::PluginSpec;
using config::PluginValue;
using config::Slot;
using Catch::Matchers::ContainsSubstring;

namespace {

// ── 两个测试用接口：形状与画笔。画笔带一个形状子槽，用来测嵌套 ──

struct Shape {
    static constexpr std::string_view kInterfaceName = "Shape";
    virtual ~Shape() = default;
    [[nodiscard]] virtual double area() const = 0;
};

struct Square final : Shape {
    static constexpr std::string_view kTypeName = "square";
    struct Config {
        double side_m = 1.0;
        template <class V>
        void visit(V& v) {
            v("side_m", side_m, "边长（米）");
        }
    };
    explicit Square(const Config& c) : side_m(c.side_m) {
        if (!(c.side_m > 0.0)) throw std::invalid_argument("side_m 必须为正");
    }
    [[nodiscard]] double area() const override { return side_m * side_m; }
    double side_m;
};

struct Disk final : Shape {
    static constexpr std::string_view kTypeName = "disk";
    struct Config {
        double radius_m = 1.0;
        template <class V>
        void visit(V& v) {
            v("radius_m", radius_m, "半径（米）");
        }
    };
    explicit Disk(const Config& c) : radius_m(c.radius_m) {}
    [[nodiscard]] double area() const override { return 3.0 * radius_m * radius_m; }
    double radius_m;
};

/// 不带任何参数的实现。
struct Dot final : Shape {
    static constexpr std::string_view kTypeName = "dot";
    struct Config {
        template <class V>
        void visit(V& /*v*/) {}
    };
    explicit Dot(const Config& /*c*/) {}
    [[nodiscard]] double area() const override { return 0.0; }
};

struct Painter {
    static constexpr std::string_view kInterfaceName = "Painter";
    virtual ~Painter() = default;
    [[nodiscard]] virtual double inkUsed() const = 0;
};

struct Brush final : Painter {
    static constexpr std::string_view kTypeName = "brush";
    struct Config {
        bool wet = false;
        std::int32_t strokes = 3;
        std::uint16_t width = 2;
        float pressure = 0.5F;
        std::string color = "red";
        Slot<Shape> stamp = Slot<Shape>::of<Square>();
        template <class V>
        void visit(V& v) {
            v("wet", wet, "湿笔");
            v("strokes", strokes, "笔画数");
            v("width", width, "笔宽");
            v("pressure", pressure, "压力");
            v("color", color, "颜色");
            v("stamp", stamp, "印章形状");
        }
    };
    explicit Brush(const Config& c) : config(c), shape(c.stamp.make()) {}
    [[nodiscard]] double inkUsed() const override { return config.strokes * shape->area(); }
    Config config;
    std::unique_ptr<Shape> shape;
};

/// 子槽没有默认实现：YAML 里必须写。
struct Stencil final : Painter {
    static constexpr std::string_view kTypeName = "stencil";
    struct Config {
        Slot<Shape> outline;
        template <class V>
        void visit(V& v) {
            v("outline", outline, "轮廓，必须选");
        }
    };
    explicit Stencil(const Config& c) : shape(c.outline.make()) {}
    [[nodiscard]] double inkUsed() const override { return shape->area(); }
    std::unique_ptr<Shape> shape;
};

/// 和形状同名的画笔：报错提示要能区分"名字对、接口错"。
struct SquarePainter final : Painter {
    static constexpr std::string_view kTypeName = "square";
    struct Config {
        template <class V>
        void visit(V& /*v*/) {}
    };
    explicit SquarePainter(const Config& /*c*/) {}
    [[nodiscard]] double inkUsed() const override { return 1.0; }
};

static_assert(PluginImplementation<Square, Shape>);
static_assert(!PluginImplementation<Square, Painter>, "不继承接口就登记不进它的目录");
static_assert(PluginImplementation<Brush, Painter>);

const std::string kPath = "modules.odometry.plugin";

PluginSpec spec(std::string type, std::map<std::string, PluginValue> params = {},
                std::map<std::string, PluginSpec> children = {}) {
    return PluginSpec{
        .type = std::move(type), .params = std::move(params), .children = std::move(children)};
}

/// 测试用的目录：三种形状、两种画笔。还原的节点都挂在配置里的同一个位置 kPath。
struct Fixture {
    Catalogs catalogs;

    Fixture() {
        catalogs.of<Shape>().add<Square>();
        catalogs.of<Shape>().add<Disk>();
        catalogs.of<Shape>().add<Dot>();
        catalogs.of<Painter>().add<Brush>();
        catalogs.of<Painter>().add<Stencil>();
    }

    [[nodiscard]] Slot<Shape> shape(const PluginSpec& s) const {
        return catalogs.resolve<Shape>(s, kPath);
    }
    [[nodiscard]] Slot<Painter> painter(const PluginSpec& s) const {
        return catalogs.resolve<Painter>(s, kPath);
    }
};

/// 期望抛 ConfigError，且消息里含全部给定片段。
template <class F>
void requireConfigError(F&& f, std::initializer_list<std::string_view> parts) {
    try {
        (void)f();
    } catch (const ConfigError& e) {
        const std::string msg = e.what();
        INFO(msg);
        for (const auto part : parts) CHECK_THAT(msg, ContainsSubstring(std::string(part)));
        return;
    }
    FAIL("没有抛 ConfigError");
}

}  // namespace

TEST_CASE("按类型名选实现，没写的参数取实现的默认值", "[config][plugin]") {
    const Fixture f;

    const Slot<Shape> slot = f.shape(spec("square"));
    REQUIRE_FALSE(slot.empty());
    CHECK(slot.type() == "square");
    CHECK(slot.make()->area() == 1.0);
    // 还原后的参数补齐了默认值：落盘的 resolved 配置据此写出每个实现的全部参数
    CHECK(slot.resolved() == spec("square", {{"side_m", 1.0}}));

    CHECK(f.shape(spec("disk", {{"radius_m", 2.0}})).make()->area() == 12.0);
}

TEST_CASE("整数写给浮点字段是同一个量", "[config][plugin]") {
    const Fixture f;
    const auto slot = f.shape(spec("square", {{"side_m", std::int64_t{3}}}));
    CHECK(slot.make()->area() == 9.0);
    CHECK(slot.resolved().params.at("side_m") == PluginValue{3.0});
}

TEST_CASE("make 每次构造一个新实例", "[config][plugin]") {
    const Fixture f;
    const auto slot = f.shape(spec("square"));
    const auto a = slot.make();
    const auto b = slot.make();
    CHECK(a.get() != b.get());

    const Slot<Shape> copy = slot;  // 值类型：拷贝出来的槽构造同样的实现
    CHECK(copy.make()->area() == a->area());
}

TEST_CASE("类型名不在槽要求的接口里：报错带路径、提示与可选项", "[config][plugin]") {
    Fixture f;

    requireConfigError([&] { return f.shape(spec("triangle")); },
                       {"modules.odometry.plugin.type = triangle", "这里要求 Shape",
                        "满足的实现有 disk、dot、square"});

    // 名字在别的接口的目录里：提示是选错了接口，而不是拼错了名字
    requireConfigError([&] { return f.painter(spec("disk")); },
                       {"这里要求 Painter", "disk 是 Shape 的实现", "brush、stencil"});

    f.catalogs.of<Painter>().add<SquarePainter>();
    // 两个接口都有 square 时各选各的，互不影响
    CHECK(f.shape(spec("square")).make()->area() == 1.0);
    CHECK(f.painter(spec("square")).make()->inkUsed() == 1.0);
}

TEST_CASE("没写 type：列出可选的实现", "[config][plugin]") {
    const Fixture f;
    requireConfigError([&] { return f.shape(spec("")); },
                       {"modules.odometry.plugin.type", "没有写 type", "disk、dot、square"});
}

TEST_CASE("未知参数是错误，提示可用的参数", "[config][plugin]") {
    const Fixture f;
    requireConfigError(
        [&] { return f.shape(spec("square", {{"sides", 2.0}})); },
        {"modules.odometry.plugin.sides", "square 没有这个参数", "可用的有 side_m"});
    // 别的实现的参数也是未知键：换了 type，旧实现的参数不会混进来
    requireConfigError([&] { return f.shape(spec("square", {{"radius_m", 2.0}})); },
                       {"radius_m", "可用的有 side_m"});
    requireConfigError([&] { return f.shape(spec("dot", {{"size", 1.0}})); },
                       {"dot 没有这个参数", "它不接受任何参数"});
    // 当成子槽写的未知键同样报
    requireConfigError([&] { return f.shape(spec("square", {}, {{"inner", spec("dot")}})); },
                       {"modules.odometry.plugin.inner", "没有这个参数"});
}

TEST_CASE("类型不符是错误，不做隐式转换", "[config][plugin]") {
    const Fixture f;
    requireConfigError([&] { return f.shape(spec("square", {{"side_m", std::string("2")}})); },
                       {"modules.odometry.plugin.side_m", "应为 double，实际是 string"});
    requireConfigError([&] { return f.painter(spec("brush", {{"wet", std::int64_t{1}}})); },
                       {"wet", "应为 bool，实际是 int"});
    requireConfigError([&] { return f.painter(spec("brush", {{"strokes", 2.5}})); },
                       {"strokes", "应为 int，实际是 double"});
    requireConfigError([&] { return f.painter(spec("brush", {{"color", true}})); },
                       {"color", "应为 string，实际是 bool"});
}

TEST_CASE("数值超出字段的取值范围是错误", "[config][plugin]") {
    const Fixture f;
    const auto brush = [&](std::string key, PluginValue v) {
        return f.painter(spec("brush", {{std::move(key), std::move(v)}}));
    };
    const std::int64_t big = std::int64_t{1} << 40;
    const double inf = std::numeric_limits<double>::infinity();
    requireConfigError([&] { return brush("strokes", big); }, {"strokes", "超出"});
    requireConfigError([&] { return brush("strokes", -big); }, {"strokes", "超出"});
    requireConfigError([&] { return brush("width", std::int64_t{-1}); }, {"width", "超出"});
    requireConfigError([&] { return brush("width", std::int64_t{65536}); }, {"width", "超出"});
    requireConfigError([&] { return brush("pressure", 1e300); }, {"pressure", "超出"});
    requireConfigError([&] { return brush("pressure", inf); }, {"pressure", "不是有限数"});

    // 边界上的值照常接受
    const PluginValue max_width{std::int64_t{65535}};
    const PluginValue negative{std::int64_t{-5}};
    CHECK(brush("width", max_width).resolved().params.at("width") == max_width);
    CHECK(brush("strokes", negative).resolved().params.at("strokes") == negative);
}

TEST_CASE("实现构造函数里的取值检查在装配时就报", "[config][plugin]") {
    const Fixture f;
    requireConfigError([&] { return f.shape(spec("square", {{"side_m", -1.0}})); },
                       {"modules.odometry.plugin", "square", "side_m 必须为正"});
}

TEST_CASE("子槽递归还原，报错路径一路带下去", "[config][plugin]") {
    const Fixture f;

    SECTION("没写子槽就用方案给的默认实现") {
        const auto slot = f.painter(spec("brush"));
        CHECK(slot.make()->inkUsed() == 3.0);
        CHECK(slot.resolved().children.at("stamp") == spec("square", {{"side_m", 1.0}}));
    }
    SECTION("写了子槽就按它选") {
        const auto slot = f.painter(spec("brush", {{"strokes", std::int64_t{2}}},
                                         {{"stamp", spec("disk", {{"radius_m", 1.0}})}}));
        CHECK(slot.make()->inkUsed() == 6.0);
    }
    SECTION("子槽里的错带完整路径") {
        const PluginSpec bad_param =
            spec("brush", {}, {{"stamp", spec("square", {{"side", 1.0}})}});
        requireConfigError([&] { return f.painter(bad_param); },
                           {"modules.odometry.plugin.stamp.side", "square 没有这个参数"});
        const PluginSpec wrong_interface = spec("brush", {}, {{"stamp", spec("brush")}});
        requireConfigError([&] { return f.painter(wrong_interface); },
                           {"modules.odometry.plugin.stamp.type = brush", "这里要求 Shape",
                            "brush 是 Painter 的实现"});
    }
    SECTION("没有默认实现的子槽必须写") {
        requireConfigError([&] { return f.painter(spec("stencil")); },
                           {"modules.odometry.plugin.outline", "缺少这个槽"});
        const auto slot = f.painter(spec("stencil", {}, {{"outline", spec("disk")}}));
        CHECK(slot.make()->inkUsed() == 3.0);
    }
    SECTION("槽写成标量、参数写成映射都是错误") {
        const PluginSpec scalar_slot = spec("brush", {{"stamp", std::string("disk")}});
        requireConfigError([&] { return f.painter(scalar_slot); },
                           {"modules.odometry.plugin.stamp", "这是一个槽"});
        requireConfigError([&] { return f.painter(spec("brush", {}, {{"color", spec("disk")}})); },
                           {"modules.odometry.plugin.color", "这是一个参数"});
    }
}

TEST_CASE("规范化导出：还原的结果再还原一次不变", "[config][plugin]") {
    const Fixture f;
    const auto first =
        f.painter(spec("brush", {{"wet", true}, {"pressure", 0.25}}, {{"stamp", spec("disk")}}));
    const PluginSpec& resolved = first.resolved();

    CHECK(resolved == spec("brush",
                           {{"wet", true},
                            {"strokes", std::int64_t{3}},
                            {"width", std::int64_t{2}},
                            {"pressure", 0.25},
                            {"color", std::string("red")}},
                           {{"stamp", spec("disk", {{"radius_m", 1.0}})}}));
    CHECK(f.painter(resolved).resolved() == resolved);
}

TEST_CASE("Slot::of 在 C++ 里直接选实现", "[config][plugin]") {
    const auto slot = Slot<Shape>::of<Square>({.side_m = 2.0});
    CHECK(slot.type() == "square");
    CHECK(slot.resolved() == spec("square", {{"side_m", 2.0}}));
    CHECK(slot.make()->area() == 4.0);

    const Slot<Shape> empty;
    CHECK(empty.empty());
    requireConfigError([&] { return empty.make(); }, {"槽是空的", "Shape"});
}

TEST_CASE("目录：名字重复是错误，缺目录要说清", "[config][plugin]") {
    Fixture f;
    requireConfigError([&] { f.catalogs.of<Shape>().add<Square>(); },
                       {"Shape 的目录里已经有 square"});
    // 用户实现可以换个名字登记同一个类
    f.catalogs.of<Shape>().add<Square>("square_alias");
    CHECK(f.shape(spec("square_alias")).type() == "square_alias");

    const Catalogs none;
    requireConfigError([&] { return none.resolve<Shape>(spec("square"), kPath); },
                       {"没有 Shape 接口的目录"});
}

TEST_CASE("schema：按接口、类型名排序，字段带种类、默认值与说明", "[config][plugin]") {
    const Fixture f;
    const auto schemas = f.catalogs.schemas();
    REQUIRE(schemas.size() == 5);
    CHECK(schemas[0].interface_name == "Painter");
    CHECK(schemas[0].type == "brush");
    CHECK(schemas[1].type == "stencil");
    CHECK(schemas[2].interface_name == "Shape");
    CHECK(schemas[2].type == "disk");

    const auto& brush = schemas[0].fields;
    REQUIRE(brush.size() == 6);
    CHECK(brush[0].name == "wet");
    CHECK(brush[0].kind == "bool");
    CHECK(brush[0].default_value == "false");
    CHECK(brush[1].kind == "int");
    CHECK(brush[1].default_value == "3");
    CHECK(brush[3].kind == "float");  // 单精度要单独标出：赋值时就舍入了
    CHECK(brush[3].default_value == "0.5");
    CHECK(brush[4].kind == "string");
    CHECK(brush[4].default_value == "red");  // 与配置 schema 同一口径：字符串原样写
    CHECK(brush[5].kind == "slot:Shape");
    CHECK(brush[5].default_value == "square");
    CHECK(brush[5].doc == "印章形状");
    // 子槽的默认节点是完整的：类型加补齐的参数，按 schema 还原的一方照抄即可
    CHECK(brush[5].default_spec == spec("square", {{"side_m", 1.0}}));

    CHECK(schemas[1].fields[0].default_value.empty());  // 没有默认实现的子槽
    CHECK(schemas[1].fields[0].default_spec == PluginSpec{});
}

TEST_CASE("单精度字段：还原后的节点存舍入后的值", "[config][plugin]") {
    const Fixture f;
    const auto slot = f.painter(spec("brush", {{"pressure", 0.1}}));
    const auto& pressure = slot.resolved().params.at("pressure");
    REQUIRE(std::holds_alternative<double>(pressure));
    CHECK(std::get<double>(pressure) == static_cast<double>(0.1F));
    CHECK(std::get<double>(pressure) != 0.1);
}

TEST_CASE("算法库的内置实现都登记在目录里，默认配置可还原", "[config][plugin][algo]") {
    Catalogs catalogs;
    algo::registerBuiltins(catalogs);
    const auto filter = [&](const PluginSpec& s) {
        return catalogs.resolve<algo::ScanFilter>(s, kPath);
    };
    const auto map = [&](const PluginSpec& s) {
        return catalogs.resolve<algo::LinearizableMap>(s, kPath);
    };

    CHECK(catalogs.of<algo::ScanFilter>().names() ==
          std::vector<std::string>{"none", "voxel_downsample"});
    CHECK(catalogs.of<algo::LinearizableMap>().names() ==
          std::vector<std::string>{"voxel_point_map"});

    CHECK(filter(spec("voxel_downsample", {{"voxel_size_m", 0.25}})).make()->typeName() ==
          "voxel_downsample");
    CHECK(map(spec("voxel_point_map")).make()->empty());

    requireConfigError([&] { return filter(spec("voxel_downsample", {{"voxel_size_m", 0.0}})); },
                       {"modules.odometry.plugin", "voxel_downsample", "voxel_size_m"});
    requireConfigError([&] { return filter(spec("voxel_point_map")); },
                       {"这里要求 ScanFilter", "voxel_point_map 是 LinearizableMap 的实现"});
    requireConfigError([&] { algo::registerBuiltins(catalogs); }, {"已经有"});
}
