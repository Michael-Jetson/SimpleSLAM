// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 模块的用法：话题、服务、参数、定时器、回调写法、句柄、异常策略、装配与观测。

#include <SimpleSLAM/comm/runtime.hpp>

#include <catch2/catch_test_macros.hpp>

#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace cm = simpleslam::comm;
using namespace std::chrono_literals;

struct Msg {
    int id = 0;
};
struct Other {
    int id = 0;
};

/// 同步的日志组件：记录交给 `tap`，终端上什么都不印。
static std::shared_ptr<simpleslam::logging::Router> capturingRouter(
    std::function<void(const simpleslam::logging::Record&)> tap,
    std::vector<simpleslam::logging::LevelRule> rules = {}) {
    static std::ostringstream quiet;
    simpleslam::logging::LogOptions options;
    options.asynchronous = false;
    options.out = &quiet;
    options.err = &quiet;
    options.rules = std::move(rules);
    options.tap = std::move(tap);
    return simpleslam::logging::Router::create(std::move(options));
}

static cm::RuntimeOptions offline() {
    cm::RuntimeOptions o;
    o.offline = true;
    return o;
}

class Recorder : public cm::Module {
public:
    explicit Recorder(std::size_t queue_size = 0) : queue_size_(queue_size) {}
    std::vector<int> order;
    std::atomic<int> count{0};
    void onInit() override {
        subscribe("/a", queue_size_, &Recorder::onA);
        subscribe("/b", queue_size_, &Recorder::onB);
    }
    void onA(const Msg& m) {
        order.push_back(m.id);
        ++count;
    }
    void onB(const Other& m) {
        order.push_back(m.id);
        ++count;
    }

private:
    std::size_t queue_size_;
};

TEST_CASE("话题：FIFO、queue_size 三种语义、latch、类型校验", "[comm][module]") {
    // 发布不在调用栈内执行回调；跨话题按发布顺序
    {
        cm::Runtime rt(offline());
        auto& r = rt.addModule<Recorder>("/r");
        rt.init();
        auto a = rt.inputPublisher<Msg>("/a");
        auto b = rt.inputPublisher<Other>("/b");
        a.publish(Msg{1});
        b.publish(Other{2});
        a.publish(Msg{3});
        CHECK(r.order.empty());
        rt.waitIdle();
        CHECK((r.order == std::vector<int>{1, 2, 3}));
    }
    // queue_size=0 无界不丢
    {
        cm::Runtime rt(offline());
        std::vector<std::string> reports;
        rt.setErrorReporter([&](std::string_view, std::string_view what) { reports.emplace_back(what); });
        auto& r = rt.addModule<Recorder>("/r", std::size_t{0});
        rt.init();
        auto a = rt.inputPublisher<Msg>("/a");
        for (int i = 0; i < 1000; ++i) a.publish(Msg{i});
        CHECK(reports.size() == 2);  // 256、512 各提醒一次
        rt.waitIdle();
        CHECK(r.order.size() == 1000);
        const auto stats = rt.stats();
        CHECK(stats.subscriptions[0].delivered == 1000);
        CHECK(stats.subscriptions[0].dropped == 0);
        CHECK(stats.subscriptions[0].high_water == 1000);
    }
    // queue_size=N 丢最旧，1 只留最新
    {
        cm::Runtime rt(offline());
        auto& two = rt.addModule<Recorder>("/two", std::size_t{2});
        auto& one = rt.addModule<Recorder>("/one", std::size_t{1});
        rt.init();
        auto a = rt.inputPublisher<Msg>("/a");
        for (int i = 0; i < 5; ++i) a.publish(Msg{i});
        rt.waitIdle();
        CHECK((two.order == std::vector<int>{3, 4}));
        CHECK((one.order == std::vector<int>{4}));
        CHECK(rt.stats().subscriptions[0].dropped == 3);
    }
    // latch 补发走订阅方的收件箱
    {
        cm::Runtime rt(offline());
        auto pub = cm::Publisher<Msg>(rt.core().topic<Msg>("/a"), cm::PublishOptions{true, false});
        pub.publish(Msg{7});
        auto& r = rt.addModule<Recorder>("/r");
        rt.init();
        CHECK(r.order.empty());
        rt.waitIdle();
        CHECK((r.order == std::vector<int>{7}));
    }
    // 类型不符是装配错误
    {
        cm::Runtime rt(offline());
        rt.core().topic<Msg>("/x");
        bool threw = false;
        try {
            rt.core().topic<Other>("/x");
        } catch (const cm::WiringError&) {
            threw = true;
        }
        CHECK(threw);
    }
}

// 自定义消息体：任意 C++ 结构体，不需要 .msg 和代码生成
struct PointCloudMsg {
    std::int64_t stamp_ns = 0;
    std::string frame_id;
    std::vector<double> xyz;
};
struct MoveOnlyMsg {
    std::unique_ptr<int> payload;
};

class CloudUser : public cm::Module {
public:
    cm::MsgPtr<PointCloudMsg> held;
    int moved = 0;
    void onInit() override {
        subscribe("/cloud", 0, &CloudUser::onCloud);
        subscribe("/moved", 0, &CloudUser::onMoved);
    }
    void onCloud(const cm::MsgPtr<PointCloudMsg>& m) { held = m; }
    void onMoved(const MoveOnlyMsg& m) { moved = *m.payload; }
};

TEST_CASE("自定义消息体：任意结构体，扇出不拷贝，只能移动的类型也能发", "[comm][module]") {
    cm::Runtime rt(offline());
    auto& a = rt.addModule<CloudUser>("/a");
    auto& b = rt.addModule<CloudUser>("/b");
    rt.init();
    auto cloud = rt.inputPublisher<PointCloudMsg>("/cloud");
    auto moved = rt.inputPublisher<MoveOnlyMsg>("/moved");
    auto msg = std::make_shared<const PointCloudMsg>(PointCloudMsg{42, "lidar", std::vector<double>(300000, 1.0)});
    cloud.publish(msg);
    MoveOnlyMsg m;
    m.payload = std::make_unique<int>(9);
    moved.publish(std::move(m));
    rt.waitIdle();
    CHECK(a.held.get() == msg.get());  // 扇出不拷贝
    CHECK(b.held.get() == msg.get());
    CHECK((a.moved == 9 && b.moved == 9));
}

class Thrower : public cm::Module {
public:
    int seen = 0;
    void onInit() override { subscribe("/a", 0, &Thrower::on); }
    void on(const Msg& m) {
        ++seen;
        if (m.id == 1) throw std::runtime_error("坏帧");
    }
};

struct AddReq {
    int a = 0;
    int b = 0;
};
struct AddRes {
    int sum = 0;
};

// 回调的几种写法
static int g_free_ref = 0;
static int g_free_ptr = 0;
static int g_free_timer = 0;
static void freeRef(const Msg& m) { g_free_ref += m.id; }
static void freePtr(const cm::MsgPtr<Msg>& m) { g_free_ptr += m->id; }
static void freeTimer() { ++g_free_timer; }
static AddRes freeAdd(const AddReq& r) { return {r.a + r.b}; }

class AllStyles : public cm::Module {
public:
    int member_ref = 0;
    int member_ptr = 0;
    int lambda_ref = 0;
    int lambda_ptr = 0;
    long use_count_in_callback = 0;
    void onInit() override {
        subscribe("/a", 0, &AllStyles::onRef);
        subscribe("/a", 0, &AllStyles::onPtr);
        subscribe("/a", 0, freeRef);
        subscribe("/a", 0, freePtr);
        subscribe<Msg>("/a", 0, freeRef);  // 显式写类型也可以
        subscribe<Msg>("/a", 0, [this](const Msg& m) { lambda_ref += m.id; });
        subscribe<Msg>("/a", 0, [this](const cm::MsgPtr<Msg>& m) {
            lambda_ptr += m->id;
            use_count_in_callback = m.use_count();
        });
        createTimer(10ms, freeTimer);
        advertiseService("/add_free", freeAdd);
        advertiseService<AddReq, AddRes>("/add_lambda", [](const AddReq& r) { return AddRes{r.a * r.b}; });
    }
    void onRef(const Msg& m) { member_ref += m.id; }
    void onPtr(const cm::MsgPtr<Msg>& m) { member_ptr += m->id; }
};

TEST_CASE("回调：成员函数、普通函数、lambda，都能取引用或消息指针", "[comm][module]") {
    g_free_ref = g_free_ptr = g_free_timer = 0;
    {
        cm::Runtime rt(offline());
        auto& s = rt.addModule<AllStyles>("/s");
        rt.init();
        rt.inputPublisher<Msg>("/a").publish(Msg{5});
        rt.spinFor(25ms);
        rt.waitIdle();
        CHECK(s.member_ref == 5);
        CHECK(s.member_ptr == 5);
        CHECK(g_free_ref == 10);  // 登记了两次
        CHECK(g_free_ptr == 5);
        CHECK(s.lambda_ref == 5);
        CHECK(s.lambda_ptr == 5);
        CHECK(g_free_timer == 2);
    }
    // 不写模块：main 里直接用运行时自带的节点
    {
        cm::Runtime rt;
        rt.addModule<AllStyles>("/s");
        std::atomic<int> seen{0};
        std::atomic<bool> in_runtime{false};
        rt.node().subscribe<Msg>("/a", 0, [&](const Msg& m) {
            seen += m.id;
            in_runtime = cm::inRuntimeThread();
        });
        rt.init();
        std::thread driver([&] { rt.spin(); });
        auto in = rt.inputPublisher<Msg>("/a");
        in.publish(Msg{3});
        auto add = rt.serviceClient<AddReq, AddRes>("/add_free");
        auto mul = rt.serviceClient<AddReq, AddRes>("/add_lambda");
        CHECK(add.call(AddReq{2, 3}, 2s).get().sum == 5);
        CHECK(mul.call(AddReq{2, 3}, 2s).get().sum == 6);
        CHECK(rt.waitIdle(2s));
        rt.requestStop();
        driver.join();
        rt.shutdown();
        CHECK(seen == 3);
        CHECK(in_runtime);
    }
}

class Handles : public cm::Module {
public:
    cm::Subscriber sub;
    cm::ServiceServer srv;
    std::atomic<int> seen{0};
    int ignored_seen = 0;
    void onInit() override {
        subscribe("/ignored", 0, &Handles::onIgnored);  // 不保存，照样收得到
        advertiseService("/h_unsaved", &Handles::add);  // 不保存，照样在线
        sub = subscribe("/a", 0, &Handles::on);
        srv = advertiseService("/h_add", &Handles::add);
    }
    void on(const Msg&) { ++seen; }
    void onIgnored(const Msg&) { ++ignored_seen; }
    AddRes add(const AddReq& r) { return {r.a + r.b}; }
};

TEST_CASE("订阅者和服务端对象：不保存也有效，可退订，可比运行时活得久", "[comm][module]") {
    {
        cm::Runtime rt(offline());
        auto& h = rt.addModule<Handles>("/h");
        auto unsaved = rt.node().serviceClient<AddReq, AddRes>("/h_unsaved");
        auto saved = rt.node().serviceClient<AddReq, AddRes>("/h_add");
        rt.init();
        auto a = rt.inputPublisher<Msg>("/a");
        auto ignored = rt.inputPublisher<Msg>("/ignored");
        a.publish(Msg{1});
        ignored.publish(Msg{1});
        rt.waitIdle();
        CHECK(h.seen == 1);
        CHECK(h.ignored_seen == 1);
        CHECK(h.sub.active());
        CHECK(h.sub.stats().delivered == 1);
        CHECK(a.subscriberCount() == 1);

        a.publish(Msg{2});  // 还在队列里
        h.sub.shutdown();
        a.publish(Msg{3});
        rt.waitIdle();
        CHECK(h.seen == 1);
        CHECK(!h.sub.active());
        CHECK(a.subscriberCount() == 0);

        cm::ServiceStatus unsaved_status = cm::ServiceStatus::kNoProvider;
        CHECK(unsaved.asyncCall(AddReq{1, 2}, 0ms, [&](cm::ServiceResult<AddRes> r) { unsaved_status = r.status; })
              == cm::ServiceStatus::kOk);
        cm::ServiceStatus queued_status = cm::ServiceStatus::kOk;
        CHECK(saved.asyncCall(AddReq{1, 2}, 0ms, [&](cm::ServiceResult<AddRes> r) { queued_status = r.status; })
              == cm::ServiceStatus::kOk);
        h.srv.shutdown();  // 已排队没执行的结案为 kStopped
        rt.waitIdle();
        CHECK(unsaved_status == cm::ServiceStatus::kOk);
        CHECK(queued_status == cm::ServiceStatus::kStopped);
        CHECK(!h.srv.active());
        CHECK(saved.call(AddReq{1, 2}, 1ms).status == cm::ServiceStatus::kNoProvider);
    }
    // 句柄比 Runtime 活得久；空句柄安全
    {
        cm::Subscriber outlive;
        {
            cm::Runtime rt(offline());
            auto& h = rt.addModule<Handles>("/h");
            rt.init();
            outlive = h.sub;
        }
        outlive.shutdown();
        CHECK(!outlive.active());
        cm::Subscriber empty;
        cm::ServiceServer empty_srv;
        empty.shutdown();
        empty_srv.shutdown();
        CHECK(!empty.active());
        CHECK(!empty_srv.active());
    }
    // 在线：发布和退订同时发生
    {
        cm::Runtime rt;
        auto& h = rt.addModule<Handles>("/h");
        rt.init();
        std::thread driver([&] { rt.spin(); });
        auto a = rt.inputPublisher<Msg>("/a");
        std::thread publisher([&] { for (int i = 0; i < 20000; ++i) a.publish(Msg{i}); });
        while (h.seen.load() == 0) std::this_thread::yield();
        h.sub.shutdown();
        const int after = h.seen.load();
        publisher.join();
        CHECK(rt.waitIdle(5s));
        CHECK(h.seen.load() <= after + 1);  // 至多还差正在执行的那一次
        rt.requestStop();
        driver.join();
        rt.shutdown();
    }
}

static std::vector<std::string> g_order;

class Traced : public cm::Module {
public:
    Traced(std::string tag, std::vector<std::string> in, std::vector<std::string> out)
        : tag_(std::move(tag)), in_(std::move(in)), out_(std::move(out)) {}
    void onInit() override {
        for (const auto& o : out_) pubs_.push_back(advertise<Msg>(o));
        for (const auto& i : in_) {
            subscribe<Msg>(i, 0, [this](const Msg& m) {
                g_order.push_back(tag_ + std::to_string(m.id));
                for (auto& p : pubs_) p.publish(m);
            });
        }
    }

private:
    std::string tag_;
    std::vector<std::string> in_, out_;
    std::vector<cm::Publisher<Msg>> pubs_;
};

class Filter : public cm::Module {
public:
    int started_on_own_thread = 0;
    int nice_seen = 0;
    std::size_t cpus_seen = 0;
    int oneshot = 0;
    std::atomic<int> count{0};
    void onInit() override {
        pub_ = advertise<Msg>("out");
        subscribe("in", 0, &Filter::on);
        createOneShot(5ms, &Filter::fire);
    }
    void onStart() override {
        started_on_own_thread = cm::inRuntimeThread() ? 1 : 0;
        nice_seen = getpriority(PRIO_PROCESS, static_cast<id_t>(syscall(SYS_gettid)));
        cpu_set_t set;
        CPU_ZERO(&set);
        sched_getaffinity(0, sizeof(set), &set);
        cpus_seen = static_cast<std::size_t>(CPU_COUNT(&set));
    }
    void fire() { ++oneshot; }
    void on(const Msg& m) {
        pub_.publish(m);
        ++count;
        log_.info("转发一条");
        log_.every(1s).warn("每秒一次");
    }

private:
    cm::Publisher<Msg> pub_;
};

class Listener : public cm::Module {
public:
    std::atomic<int> count{0};
    void onInit() override { subscribe("/imu", 0, &Listener::on); }
    void on(const Msg&) { ++count; }
};

static std::size_t threadCount() {
    std::size_t n = 0;
    for (const auto& entry : std::filesystem::directory_iterator("/proc/self/task")) {
        (void)entry;
        ++n;
    }
    return n;
}

/// join 返回之后，刚退出的线程还会在 /proc 里留一小会儿。
static bool threadCountSettlesAt(std::size_t expected) {
    for (int i = 0; i < 2000; ++i) {
        if (threadCount() == expected) return true;
        std::this_thread::sleep_for(1ms);
    }
    return false;
}

class TimedModule : public cm::Module {
public:
    std::atomic<int> ticks{0};
    std::atomic<int> oneshots{0};
    std::atomic<int> messages{0};
    std::atomic<std::uint64_t> tick_thread{0};
    std::atomic<std::uint64_t> message_thread{0};
    void onInit() override {
        createTimer(1ms, &TimedModule::tick);
        createOneShot(3ms, &TimedModule::once);
        subscribe("/a", 0, &TimedModule::on);
    }
    void tick() {
        ++ticks;
        tick_thread = std::hash<std::thread::id>{}(std::this_thread::get_id());
    }
    void once() { ++oneshots; }
    void on(const Msg&) {
        ++messages;
        message_thread = std::hash<std::thread::id>{}(std::this_thread::get_id());
    }
    void lateTimer() { createOneShot(1ms, &TimedModule::once); }
};

TEST_CASE("通信层自身不占线程：定时器在模块线程上，线程数等于模块数", "[comm][module]") {
    // 通信层自身不占线程：start() 之后多出来的线程数恰好等于模块数
    {
        // 等之前用例退出的线程从 /proc 消失。TSAN 自己还有一个后台线程，所以不能写死成 1；
        // 它在进程第一次建线程时才出现，先起停一个线程，让它在取基数之前就出现
        std::thread([] {}).join();
        std::size_t before = threadCount();
        for (int stable = 0, i = 0; stable < 50 && i < 2000; ++i) {
            std::this_thread::sleep_for(1ms);
            const std::size_t now = threadCount();
            stable = now == before ? stable + 1 : 0;
            before = now;
        }
        cm::Runtime rt;
        auto& a = rt.addModule<TimedModule>("/a_mod");
        rt.addModule<TimedModule>("/b_mod");
        rt.addModule<Recorder>("/c_mod");
        auto in = rt.inputPublisher<Msg>("/a");
        rt.init();
        rt.start();
        CHECK(threadCount() == before + 3);
        in.publish(Msg{1});
        for (int i = 0; i < 2000 && (a.ticks.load() < 5 || a.oneshots.load() < 1 || a.messages.load() < 1); ++i) {
            std::this_thread::sleep_for(1ms);
        }
        // 定时器和消息都在模块自己的线程上
        CHECK(a.ticks.load() >= 5);
        CHECK(a.oneshots.load() == 1);
        CHECK(a.messages.load() == 1);
        CHECK(a.tick_thread.load() == a.message_thread.load());
        // 运行中登记更早的定时器，线程要按新的时刻重新等待
        a.lateTimer();
        for (int i = 0; i < 2000 && a.oneshots.load() < 2; ++i) std::this_thread::sleep_for(1ms);
        CHECK(a.oneshots.load() == 2);
        rt.requestStop();
        rt.shutdown();
        CHECK(rt.state() == cm::RuntimeState::kDone);
        CHECK(threadCountSettlesAt(before));
    }
    // 只有定时器、没有消息时，等待不空转
    {
        cm::Runtime rt;
        auto& t = rt.addModule<TimedModule>("/t");
        rt.init();
        rt.start();
        std::this_thread::sleep_for(30ms);
        const int ticks = t.ticks.load();
        CHECK(ticks >= 10);
        CHECK(ticks <= 35);
        rt.shutdown();
    }
    // 服务超时在调用方模块的定时器上（在线）
    {
        class Busy : public cm::Module {
        public:
            void onInit() override { advertiseService<AddReq, AddRes>("/busy", &Busy::add); }
            AddRes add(const AddReq&) {
                std::this_thread::sleep_for(100ms);
                return {};
            }
        };
        cm::Runtime rt;
        rt.addModule<Busy>("/busy_mod");
        auto client = rt.node().serviceClient<AddReq, AddRes>("/busy");
        std::atomic<int> status{-1};
        rt.init();
        rt.start();
        std::this_thread::sleep_for(20ms);  // 让模块线程先进入等待：新定时器要能叫醒它
        CHECK(client.asyncCall(AddReq{}, 10ms, [&](cm::ServiceResult<AddRes> r) { status = static_cast<int>(r.status); })
              == cm::ServiceStatus::kOk);
        for (int i = 0; i < 2000 && status.load() < 0; ++i) std::this_thread::sleep_for(1ms);
        CHECK(status.load() == static_cast<int>(cm::ServiceStatus::kTimeout));
        rt.shutdown();
    }
    // 模块线程已经睡着、没有任何定时器时，运行中登记的一次性定时器也要能叫醒它
    {
        class Idle : public cm::Module {
        public:
            std::atomic<int> fired{0};
            void arm() { createOneShot(5ms, &Idle::fire); }
            void fire() { ++fired; }
        };
        cm::Runtime rt;
        auto& idle = rt.addModule<Idle>("/idle");
        rt.init();
        rt.start();
        std::this_thread::sleep_for(20ms);
        idle.arm();
        for (int i = 0; i < 2000 && idle.fired.load() == 0; ++i) std::this_thread::sleep_for(1ms);
        CHECK(idle.fired.load() == 1);
        rt.shutdown();
    }
    // 不用节点就没有节点
    {
        cm::Runtime rt(offline());
        rt.init();
        CHECK(rt.stats().modules.empty());
    }
}

TEST_CASE("重映射、onStart、线程属性、一次性定时器、日志、统计、调度打乱", "[comm][module]") {
    // 装配完成就绪
    {
        cm::Runtime rt(offline());
        rt.addModule<Filter>("/slam/filter");
        auto in = rt.inputPublisher<Msg>("/slam/in");
        rt.init();
        CHECK(rt.state() == cm::RuntimeState::kReady);
    }
    // 重映射：模块类不变，装配时把它接到别的话题上
    {
        cm::Runtime rt(offline());
        auto& filter = rt.addModule<Filter>("/slam/filter");
        filter.remap("in", "/imu_raw");
        filter.remap("out", "/imu");
        auto& listener = rt.addModule<Listener>("/slam/listener");
        auto raw = rt.inputPublisher<Msg>("/imu_raw");
        rt.init();
        raw.publish(Msg{1});
        rt.waitIdle();
        CHECK(listener.count == 1);
        CHECK(filter.count == 1);
    }
    // 一次性定时器、onStart、日志（离线）
    {
        std::vector<std::string> logs;
        auto options = offline();
        options.log_router = capturingRouter([&](const simpleslam::logging::Record& r) {
            logs.push_back(r.source + ":" + r.text);
        });
        cm::Runtime rt(options);
        auto& filter = rt.addModule<Filter>("/slam/filter");
        auto in = rt.inputPublisher<Msg>("/slam/in");
        rt.init();
        CHECK(filter.started_on_own_thread == 0);
        rt.spinFor(20ms);
        CHECK(filter.started_on_own_thread == 1);
        CHECK(filter.oneshot == 1);
        rt.spinFor(20ms);
        CHECK(filter.oneshot == 1);
        for (int i = 0; i < 3; ++i) in.publish(Msg{i});
        rt.waitIdle();
        int info = 0, every = 0;
        for (const auto& l : logs) {
            if (l == "/slam/filter:转发一条") ++info;
            if (l == "/slam/filter:每秒一次") ++every;
        }
        CHECK(info == 3);
        CHECK(every == 1);  // 虚拟时钟没走够 1 秒
        CHECK(rt.statsTable().find("/slam/filter") != std::string::npos);
    }
    // 日志按来源设等级：关掉的等级不记
    {
        std::vector<std::string> logs;
        auto options = offline();
        options.log_router = capturingRouter(
            [&](const simpleslam::logging::Record& r) { logs.push_back(r.text); },
            {{.key = "filter", .level = simpleslam::logging::Level::kError}});
        cm::Runtime rt(options);
        rt.addModule<Filter>("/slam/filter");
        auto in = rt.inputPublisher<Msg>("/slam/in");
        rt.init();
        in.publish(Msg{9});
        rt.waitIdle();
        CHECK(logs.empty());
    }
    // 线程属性（在线）
    {
        cm::Runtime rt(cm::RuntimeOptions{.log_router = capturingRouter([](const auto&) {})});
        auto& filter = rt.addModule<Filter>("/slam/filter");
        filter.setThreadOptions({5, {0}});
        auto in = rt.inputPublisher<Msg>("/slam/in");
        rt.init();
        std::thread driver([&] { rt.spin(); });
        in.publish(Msg{1});
        CHECK(rt.waitIdle(2s));
        rt.requestStop();
        driver.join();
        rt.shutdown();
        CHECK(filter.started_on_own_thread == 1);
        CHECK(filter.nice_seen == 5);
        CHECK(filter.cpus_seen == 1);
    }
    // 调度打乱：模块内 FIFO 不变；同种子可复现；不同种子真的不一样
    {
        auto run = [](std::uint64_t seed) {
            g_order.clear();
            cm::RuntimeOptions o;
            o.offline = true;
            o.schedule_seed = seed;
            cm::Runtime rt(o);
            rt.addModule<Traced>("/a", "A", std::vector<std::string>{"/in"}, std::vector<std::string>{"/x", "/y"});
            rt.addModule<Traced>("/b", "B", std::vector<std::string>{"/x"}, std::vector<std::string>{"/z"});
            rt.addModule<Traced>("/c", "C", std::vector<std::string>{"/y", "/z"}, std::vector<std::string>{});
            rt.addModule<Traced>("/d", "D", std::vector<std::string>{"/in", "/z"}, std::vector<std::string>{});
            rt.init();
            auto in = rt.inputPublisher<Msg>("/in");
            for (int i = 0; i < 20; ++i) in.publish(Msg{i});
            rt.waitIdle();
            return g_order;
        };
        const auto fifo = run(0);
        const auto s1 = run(1);
        CHECK(s1 == run(1));
        CHECK(s1 != fifo);
        std::vector<int> a_ids;
        for (const auto& e : s1) {
            if (e[0] == 'A') a_ids.push_back(std::stoi(e.substr(1)));
        }
        CHECK(std::is_sorted(a_ids.begin(), a_ids.end()));
        CHECK(a_ids.size() == 20);
    }
}

class Critical : public Thrower {
public:
    void onInit() override {
        Thrower::onInit();
        setExceptionPolicy(cm::ExceptionPolicy::kStop);
    }
};

TEST_CASE("异常策略：隔离、停机、按模块设置、重新抛出", "[comm][module]") {
    // 默认隔离：继续处理，记下第一个异常
    {
        cm::Runtime rt(offline());
        auto& t = rt.addModule<Thrower>("/t");
        rt.init();
        auto a = rt.inputPublisher<Msg>("/a");
        for (int id : {1, 1, 2}) a.publish(Msg{id});
        rt.waitIdle();
        CHECK(t.seen == 3);
        CHECK(rt.isolatedFailures() == 2);
        const auto first = rt.firstFailure();
        CHECK((first && first->module == "/t" && first->what == "坏帧"));
    }
    // 停机（离线）：从驱动调用重新抛出；已接纳的走完；入口和参数写入都停
    {
        auto o = offline();
        o.on_exception = cm::ExceptionPolicy::kStop;
        cm::Runtime rt(o);
        auto& t = rt.addModule<Thrower>("/t");
        rt.init();
        auto a = rt.inputPublisher<Msg>("/a");
        a.publish(Msg{1});
        a.publish(Msg{2});
        std::string caught;
        try {
            rt.waitIdle();
        } catch (const std::runtime_error& e) {
            caught = e.what();
        }
        CHECK(caught == "坏帧");
        CHECK(t.seen == 2);
        a.publish(Msg{3});
        CHECK(rt.waitIdle());  // 只抛一次
        CHECK(t.seen == 2);
        CHECK(rt.setParams({}).status == cm::ParamCommit::kRejectedStopped);
        rt.shutdown();
        CHECK(rt.state() == cm::RuntimeState::kDone);
    }
    // 停机（在线）：spin 走完已接纳的工作之后重新抛出
    {
        cm::RuntimeOptions o;
        o.on_exception = cm::ExceptionPolicy::kStop;
        cm::Runtime rt(o);
        rt.addModule<Thrower>("/t");
        auto& r = rt.addModule<Recorder>("/r");
        rt.init();
        std::string caught;
        std::thread driver([&] {
            try {
                rt.spin();
            } catch (const std::exception& e) {
                caught = e.what();
            }
        });
        auto a = rt.inputPublisher<Msg>("/a");
        a.publish(Msg{7});
        a.publish(Msg{1});
        driver.join();
        CHECK(caught == "坏帧");
        CHECK(r.count == 2);
        rt.shutdown();
    }
    // 模块级：关键模块停机，其余隔离
    {
        cm::Runtime rt(offline());
        auto& critical = rt.addModule<Critical>("/critical");
        rt.init();
        auto a = rt.inputPublisher<Msg>("/a");
        a.publish(Msg{1});
        bool threw = false;
        try {
            rt.waitIdle();
        } catch (const std::runtime_error&) {
            threw = true;
        }
        CHECK(threw);
        CHECK(critical.seen == 1);
    }
    // 装配期的异常直接抛给调用方
    {
        class BadInit : public cm::Module {
        public:
            void onInit() override { throw cm::WiringError("缺配置"); }
        };
        cm::Runtime rt(offline());
        rt.addModule<BadInit>("/bad");
        bool threw = false;
        try {
            rt.init();
        } catch (const cm::WiringError&) {
            threw = true;
        }
        CHECK(threw);
    }
}

TEST_CASE("回调异常默认隔离，下一条照常处理", "[comm][module]") {
    cm::Runtime rt(offline());
    auto& t = rt.addModule<Thrower>("/t");
    rt.init();
    auto a = rt.inputPublisher<Msg>("/a");
    a.publish(Msg{1});
    a.publish(Msg{2});
    rt.waitIdle();
    CHECK(t.seen == 2);
    CHECK(rt.isolatedFailures() == 1);
}

class Ticker : public cm::Module {
public:
    int fired = 0;
    cm::TimerHandle handle;
    void onInit() override { handle = createTimer(10ms, &Ticker::tick); }
    void tick() { ++fired; }
};

TEST_CASE("虚拟时钟下的定时器与取消", "[comm][module]") {
    cm::Runtime rt(offline());
    auto& t = rt.addModule<Ticker>("/t");
    rt.init();
    rt.spinFor(35ms);
    CHECK(t.fired == 3);
    t.handle.cancel();
    rt.spinFor(50ms);
    CHECK(t.fired == 3);
}

constexpr cm::Param<double> kGain{
    .name = "t.gain", .fallback = 1.0, .doc = "增益", .min = 0.0, .runtime = true};
constexpr cm::Param<double> kLevel{.name = "t.level", .fallback = 0.0, .runtime = true};
constexpr cm::Param<double> kScale{.name = "t.scale", .fallback = 1.0, .runtime = true};

/// 插件：在所属模块的回调里读写参数，读的是模块这次回调的快照，写先记下。
class LevelPlugin : public cm::ParamUser {
public:
    [[nodiscard]] double level() const { return param(kLevel); }
    void raise() const { setParam(kLevel, param(kLevel) + 1.0); }
};

class ParamModule : public cm::Module {
public:
    double initial = 0;
    double seen_in_callback = 0;
    double level_in_callback = -1;
    double level_after_write = -1;
    bool plugin_writes = false;
    std::vector<std::uint64_t> revisions;
    LevelPlugin plugin;

    void onInit() override {
        initial = param(kGain);
        subscribe("/a", 0, &ParamModule::on);
    }
    void on(const Msg&) {
        seen_in_callback = param(kGain);
        level_in_callback = plugin.level();
        if (plugin_writes) {
            plugin.raise();
            level_after_write = plugin.level();  // 写先记下：这次回调里还是旧值
        }
    }
    void onParamsChanged(const cm::ParamSnapshot& s) override { revisions.push_back(s.revision); }
    cm::ParamCommitResult write(double v) { return setParam(kScale, v); }
    [[nodiscard]] double scale() const { return param(kScale); }
};

cm::ParamDefinition defined(const cm::Param<double>& p, std::vector<std::string> writers) {
    return cm::ParamDefinition{.name = std::string(p.name),
                               .type = cm::ParamType::kReal,
                               .fallback = p.fallback,
                               .doc = std::string(p.doc),
                               .min = p.min,
                               .max = p.max,
                               .runtime = p.runtime,
                               .writers = std::move(writers)};
}

TEST_CASE("参数服务器：装配时定义、帧边界换代、核对、模块与插件写入、变更回调", "[comm][module]") {
    cm::Runtime rt(offline());
    // gain 没有组件写，只有外部能改；level 由插件写、scale 由模块写，写者都是 /t
    rt.params().define({defined(kGain, {}), defined(kLevel, {"/t"}), defined(kScale, {"/t"})},
                       {{"t.gain", 2.0}});
    auto& t = rt.addModule<ParamModule>("/t");
    rt.init();
    CHECK(t.initial == 2.0);  // 配置给的值

    auto a = rt.inputPublisher<Msg>("/a");
    a.publish(Msg{1});
    CHECK(rt.setParams({{"t.gain", 3.0}}).ok());
    a.publish(Msg{2});
    rt.spinOnce(1);  // 一批之内：先处理消息 1，再换参数，再处理消息 2
    CHECK(t.seen_in_callback == 3.0);
    CHECK((t.revisions == std::vector<std::uint64_t>{1}));

    CHECK(rt.setParams({{"nope", 1.0}}).status == cm::ParamCommit::kUnknownName);
    CHECK(rt.setParams({{"t.gain", true}}).status == cm::ParamCommit::kRejectedType);
    CHECK(rt.setParams({{"t.gain", -1.0}}).status == cm::ParamCommit::kRejectedValue);
    // level 有组件写：外部不能改
    CHECK(rt.setParams({{"t.level", 5.0}}).status == cm::ParamCommit::kRejectedWriter);

    // 插件写：回调结束后由模块提交，下一次回调起看到新值
    t.plugin_writes = true;
    a.publish(Msg{3});
    rt.waitIdle();
    CHECK(t.level_in_callback == 0.0);
    CHECK(t.level_after_write == 0.0);
    a.publish(Msg{4});
    rt.waitIdle();
    CHECK(t.level_in_callback == 1.0);
    CHECK(rt.params().snapshot()->get<double>("t.level") == 2.0);

    CHECK(t.write(4.0).ok());  // 模块自己写，马上提交
    rt.waitIdle();
    CHECK(t.scale() == 4.0);
    rt.requestStop();
    CHECK(rt.setParams({{"t.gain", 5.0}}).status == cm::ParamCommit::kRejectedStopped);
}

class Adder : public cm::Module {
public:
    void onInit() override { advertiseService<AddReq, AddRes>("/add", &Adder::add); }
    AddRes add(const AddReq& r) {
        if (r.a < 0) throw std::runtime_error("负数");
        return {r.a + r.b};
    }
};

class Caller : public cm::Module {
public:
    cm::ServiceClient<AddReq, AddRes> client;
    std::vector<cm::ServiceStatus> results;
    int last = 0;
    cm::ServiceStatus blocking = cm::ServiceStatus::kOk;
    void onInit() override {
        client = serviceClient<AddReq, AddRes>("/add");
        subscribe("/a", 0, &Caller::on);
    }
    void on(const Msg& m) {
        const auto admitted = client.asyncCall(AddReq{m.id, 1}, 50ms, [this](cm::ServiceResult<AddRes> r) {
            results.push_back(r.status);
            if (r) last = r.value->sum;
        });
        CHECK(admitted == cm::ServiceStatus::kOk);
        blocking = client.call(AddReq{1, 2}, 1ms).status;
    }
};

class Silent : public cm::Module {
public:
    void onInit() override { advertiseService<AddReq, AddRes>("/silent", &Silent::add); }
    AddRes add(const AddReq&) { return {}; }
};

TEST_CASE("服务：异步、超时、无服务端、阻塞调用、处理函数异常原因", "[comm][module]") {
    {
        cm::Runtime rt(offline());
        rt.addModule<Adder>("/adder");
        auto& c = rt.addModule<Caller>("/caller");
        rt.init();
        auto a = rt.inputPublisher<Msg>("/a");
        a.publish(Msg{20});
        rt.waitIdle();
        CHECK((c.results.size() == 1 && c.results[0] == cm::ServiceStatus::kOk));
        CHECK(c.last == 21);
        CHECK(c.blocking == cm::ServiceStatus::kWouldDeadlock);
        a.publish(Msg{-1});
        rt.waitIdle();
        CHECK((c.results.size() == 2 && c.results[1] == cm::ServiceStatus::kHandlerError));
    }
    // 超时：服务端一直没被驱动到，时钟先走到
    {
        cm::Runtime rt(offline());
        rt.addModule<Silent>("/silent");
        auto& c = rt.addModule<Caller>("/caller");
        auto client = rt.node().serviceClient<AddReq, AddRes>("/silent");
        rt.init();
        (void)c;
        cm::ServiceStatus status = cm::ServiceStatus::kOk;
        CHECK(client.asyncCall(AddReq{}, 10ms, [&](cm::ServiceResult<AddRes> r) { status = r.status; })
              == cm::ServiceStatus::kOk);
        rt.virtualClock()->advanceTo(20ms);
        rt.waitIdle();
        CHECK(status == cm::ServiceStatus::kTimeout);
    }
    // 没有服务端
    {
        cm::Runtime rt(offline());
        rt.init();
        auto client = rt.serviceClient<AddReq, AddRes>("/nobody");
        CHECK(client.call(AddReq{}, 1ms).status == cm::ServiceStatus::kNoProvider);
    }
    // 在线：外部线程阻塞调用；关闭时未执行的请求结案为 kStopped
    {
        cm::Runtime rt;
        rt.addModule<Adder>("/adder");
        rt.init();
        std::thread driver([&] { rt.spin(); });
        auto client = rt.serviceClient<AddReq, AddRes>("/add");
        auto r = client.call(AddReq{2, 3}, 2s);
        CHECK((r.ok() && r.get().sum == 5));
        auto bad = client.call(AddReq{-1, 3}, 2s);
        CHECK((bad.status == cm::ServiceStatus::kHandlerError && bad.error == "负数"));
        std::string thrown;
        try {
            (void)bad.get();
        } catch (const cm::ServiceError& e) {
            thrown = e.what();
        }
        CHECK(thrown == "负数");
        rt.requestStop();
        driver.join();
        rt.shutdown();
        CHECK(client.call(AddReq{2, 3}, 1ms).status == cm::ServiceStatus::kStopped);
    }
}

// 同一模块的回调不并发；未加锁的成员只有串行时才安全（TSAN 会盯着）
class Serial : public cm::Module {
public:
    std::atomic<int> inside{0};
    std::atomic<int> overlaps{0};
    int unsynchronized = 0;
    std::atomic<int> total{0};
    void onInit() override {
        subscribe("/a", 0, &Serial::onA);
        subscribe("/b", 0, &Serial::onB);
    }
    void enter() {
        if (inside.fetch_add(1) != 0) ++overlaps;
        ++unsynchronized;
        inside.fetch_sub(1);
        ++total;
    }
    void onA(const Msg&) { enter(); }
    void onB(const Other&) { enter(); }
};

class Relay : public cm::Module {
public:
    Relay(std::string in, std::string out) : in_(std::move(in)), out_(std::move(out)) {}
    void onInit() override {
        pub_ = advertise<Msg>(out_);
        subscribe(in_, 0, &Relay::on);
    }
    void on(const cm::MsgPtr<Msg>& m) { pub_.publish(m); }

private:
    std::string in_, out_;
    cm::Publisher<Msg> pub_;
};

class Tail : public cm::Module {
public:
    Tail(std::string in, cm::Publisher<Msg>* unused = nullptr) : in_(std::move(in)) { (void)unused; }
    std::atomic<int> count{0};
    bool finish_seen = false;
    void onInit() override {
        subscribe(in_, 0, &Tail::on);
        subscribe("/finish", 0, &Tail::onFinishMsg);
    }
    void on(const Msg&) { ++count; }
    void onFinishMsg(const Other&) { finish_seen = true; }

private:
    std::string in_;
};

class FinishPublisher : public cm::Module {
public:
    void onInit() override { pub_ = advertise<Other>("/finish"); }
    void onFinish() override { pub_.publish(Other{1}); }

private:
    cm::Publisher<Other> pub_;
};

TEST_CASE("在线：模块内串行、停止走完因果链、取消、长回调不挡别的模块", "[comm][module]") {
    // 多生产者：模块内严格串行
    {
        cm::Runtime rt;
        auto& s = rt.addModule<Serial>("/s");
        rt.init();
        std::thread driver([&] { rt.spin(); });
        auto a = rt.inputPublisher<Msg>("/a");
        auto b = rt.inputPublisher<Other>("/b");
        std::thread p1([&] { for (int i = 0; i < 20000; ++i) a.publish(Msg{i}); });
        std::thread p2([&] { for (int i = 0; i < 20000; ++i) b.publish(Other{i}); });
        p1.join();
        p2.join();
        CHECK(rt.waitIdle(5s));
        rt.requestStop();
        driver.join();
        rt.shutdown();
        CHECK(s.total == 40000);
        CHECK(s.unsynchronized == 40000);
        CHECK(s.overlaps == 0);
    }
    // 停止：入口不再收，已接纳的走完因果链，收尾钩子发的消息也会处理
    {
        cm::Runtime rt;
        rt.addModule<Relay>("/r1", "/in", "/m1");
        rt.addModule<Relay>("/r2", "/m1", "/m2");
        auto& tail = rt.addModule<Tail>("/tail", "/m2");
        rt.addModule<FinishPublisher>("/fp");
        rt.init();
        std::thread driver([&] { rt.spin(); });
        auto in = rt.inputPublisher<Msg>("/in");
        for (int i = 0; i < 5000; ++i) in.publish(Msg{i});
        rt.requestStop();
        in.publish(Msg{-1});  // 停止之后不再发
        driver.join();
        CHECK(tail.count == 5000);
        rt.shutdown();
        CHECK(tail.finish_seen);
        CHECK(rt.shutdownDrained());
        CHECK(rt.state() == cm::RuntimeState::kDone);
    }
    // 回调里 requestStop
    {
        class Stopper : public cm::Module {
        public:
            explicit Stopper(cm::Runtime& rt) : rt_(rt) {}
            void onInit() override { subscribe("/a", 0, &Stopper::on); }
            void on(const Msg&) { rt_.requestStop(); }

        private:
            cm::Runtime& rt_;
        };
        cm::Runtime rt;
        rt.addModule<Stopper>("/stop", rt);
        rt.init();
        auto a = rt.inputPublisher<Msg>("/a");
        std::thread driver([&] { rt.spin(); });
        a.publish(Msg{1});
        driver.join();
        rt.shutdown();
        CHECK(rt.state() == cm::RuntimeState::kDone);
    }
    // 取消：排队的工作丢弃，spin 正在跑时也能结束
    {
        class Slow : public cm::Module {
        public:
            std::atomic<int> seen{0};
            void onInit() override { subscribe("/a", 0, &Slow::on); }
            void on(const Msg&) {
                ++seen;
                std::this_thread::sleep_for(20ms);
            }
        };
        cm::Runtime rt;
        auto& slow = rt.addModule<Slow>("/slow");
        rt.init();
        auto a = rt.inputPublisher<Msg>("/a");
        for (int i = 0; i < 100; ++i) a.publish(Msg{i});
        std::thread driver([&] { rt.spin(); });
        std::this_thread::sleep_for(30ms);
        rt.shutdown(cm::ShutdownMode::kCancel);
        driver.join();
        CHECK(rt.cancelled());
        CHECK(!rt.shutdownDrained());
        CHECK(slow.seen < 100);
    }
    // 长回调不挡别的模块
    {
        class Blocker : public cm::Module {
        public:
            void onInit() override { subscribe("/slow", 0, &Blocker::on); }
            void on(const Msg&) { std::this_thread::sleep_for(300ms); }
        };
        cm::Runtime rt;
        rt.addModule<Blocker>("/b1");
        rt.addModule<Blocker>("/b2");
        rt.addModule<Blocker>("/b3");
        auto& r = rt.addModule<Recorder>("/fast");
        rt.init();
        std::thread driver([&] { rt.spin(); });
        rt.inputPublisher<Msg>("/slow").publish(Msg{0});
        std::this_thread::sleep_for(20ms);
        const auto t0 = std::chrono::steady_clock::now();
        rt.inputPublisher<Msg>("/a").publish(Msg{1});
        while (r.count.load() == 0) std::this_thread::yield();
        CHECK(std::chrono::steady_clock::now() - t0 < 100ms);
        rt.requestStop();
        driver.join();
        rt.shutdown();
    }
    // 收尾排空时定时器不再触发
    {
        class Periodic : public cm::Module {
        public:
            std::atomic<int> fired{0};
            void onInit() override { createTimer(1ms, &Periodic::tick); }
            void tick() { ++fired; }
        };
        cm::Runtime rt;
        auto& p = rt.addModule<Periodic>("/p");
        rt.init();
        std::thread driver([&] { rt.spin(); });
        std::this_thread::sleep_for(20ms);
        rt.requestStop();
        driver.join();
        const int at_stop = p.fired;
        std::this_thread::sleep_for(5ms);
        rt.shutdown();
        CHECK(at_stop > 0);
        CHECK(p.fired == at_stop);
    }
    // 钩子里重入 shutdown 不等待自身，收尾只有一次
    {
        class Reenter : public cm::Module {
        public:
            explicit Reenter(cm::Runtime& rt) : rt_(rt) {}
            int finished = 0;
            void onFinish() override {
                ++finished;
                rt_.shutdown();
            }

        private:
            cm::Runtime& rt_;
        };
        cm::Runtime rt(offline());
        auto& r = rt.addModule<Reenter>("/r", rt);
        rt.init();
        rt.shutdown();
        CHECK(r.finished == 1);
        CHECK(rt.state() == cm::RuntimeState::kDone);
    }
}
