# 通信执行器设计实验

本目录是独立的选型实验，不接入项目主构建，不是生产通信层。用于验证 Asio 能否承担模块串行执行、Latest 投递、异步请求响应和参数提交的共同执行机制。

目前已在 GCC 13.3、Boost 1.83 / Boost.Asio 1.28.2 下构建并通过 CTest，未在 standalone Asio 下验证。目标依赖建议使用 standalone Asio；环境内只有 Boost.Asio，所以实验默认使用后者。下载 standalone 源码时本机代理不可达，没有修改系统依赖。

从项目根目录运行：

```bash
cmake -S tests/spikes -B /tmp/simpleslam-comm-spike-build -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/simpleslam-comm-spike-build -j2
ctest --test-dir /tmp/simpleslam-comm-spike-build --output-on-failure
```

已安装 standalone Asio 时可另用独立构建目录：

```bash
cmake -S tests/spikes -B /tmp/simpleslam-comm-spike-standalone -DSPIKE_STANDALONE_ASIO=ON -DASIO_INCLUDE_DIR=/path/to/asio/include
cmake --build /tmp/simpleslam-comm-spike-standalone -j2
ctest --test-dir /tmp/simpleslam-comm-spike-standalone --output-on-failure
```

实验检查五类行为：

- 两个发布线程、两个执行线程，共 4000 次处理；同一模块 strand 上的处理不重叠。
- Latest 只保留待处理的新值，投递不在调用栈内同步执行；回调可以再次发布和查询。
- 服务请求在服务端 strand 执行，响应回到客户端 strand。
- 超时与迟到响应只产生一次本地完成回调；明确演示超时并不撤销服务端已经接收的操作。
- 参数变更在所属 strand 校验和提交；无效变更、版本冲突被拒绝；旧快照保持不变。

局限：没有命名发现、一般 Topic 扇出、完整订阅生命周期、服务端异常封装、显式取消、模块关闭屏障、有界服务准入、虚拟时钟或跨进程传输。参数实验使用一个固定结构体，只证明提交方式，不代替生产 schema。LatestSlot 是单消费者实验，不是待直接合入的通用 Topic。

测试通过证明这些机制可以组合，不证明生产 API 已实现，也不证明一般在线多线程执行与离线回放逐位一致。现行设计见 `docs/design/comm.md`，候选取舍见 `docs/research/通信层第三方库调研_2026.md`。
