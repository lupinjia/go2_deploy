# 问题记录：引入 DataTamer 数据日志后实机不响应

> 状态：**未解决（open）**，当前已回退到不含 DataTamer 的版本（`dev_fix` 分支，基于
> `cfc54c9`），实机可正常运行。本文记录问题现象、定位过程、已尝试的修复及其结果，
> 供后续排查参考。

## 1. 问题现象

在 Unitree Go2 实机（Jetson Orin NX，arm64）上运行带 DataTamer 数据日志的控制器：

- ✅ 能正常收到机器人状态反馈（`rt/lowstate`）与遥控器数据
- ✅ 程序正常发布 low_cmd（`rt/lowcmd`），发布链路本身工作
- ❌ **机器人不执行任何命令**（不站立、不运动、无响应）

通过 git bisect 定位到首个"坏"提交：

```
864c776 [feat]：新增 DataTamer 高频数据日志（500Hz MCAP），移除旧 CSV 日志
  ↑ 引入 DataTamer 后实机不响应
cfc54c9 [feat]：切换至 onnxruntime 推理，统一为 metadata 驱动的 Go2OnnxController
  ↑ 控制器正常，实机响应正常
```

即：**onnxruntime 推理本身没问题（cfc54c9 正常），一旦叠加 DataTamer 日志就实机不响应。**

## 2. 排查历程

### 尝试 1：修复 onnxruntime 符号链接被展平（已解决，与本次问题无关）

- 现象：`make` 链接时报 `libonnxruntime.so: file format not recognized; treating as linker script`
- 原因：`core.symlinks=false` 导致 git 把符号链接以 24 字节文本文件形式检出
- 修复：CMakeLists 增加符号链接检测与重建逻辑（读 ELF magic `7f454c46` 判断），
  configure 时自动重建 `libonnxruntime.so` / `libonnxruntime.so.1`
- 结果：构建通过。**但这只是构建问题，不是实机不响应的原因。**

### 尝试 2：DataLogger 创建可能抛异常导致崩溃（已修复，无效）

- 假设：`StartControl()` 在控制线程启动**之前**创建 `DataLogger`；`MCAPSink`
  构造函数若打不开目标目录会 `throw std::runtime_error("Failed to open MCAP file
  for writing")`；`logs/` 目录被 .gitignore 忽略、实机不存在 → 异常未被捕获 →
  进程在控制线程启动前崩溃 → low_cmd 从未发送
- 验证：实测 MCAPSink 在目录不存在时确实抛异常
- 修复：创建前 `fs::create_directories` + 整个创建过程用 try/catch 包裹，失败只告警、
  继续启动控制线程
- 结果：**无效**。实机依然不响应。（说明实机上 logs 目录是存在的，或崩溃点不在这里）

### 尝试 3：命令行参数约定变化（已修复，无效）

- 假设：旧分支用 `./go2_deploy <type> <interface>`（argv[2] 是网卡），新代码改为
  `./go2_deploy <interface>`（argv[1]）。若仍按旧习惯执行，DDS 会把 `"wtw"` 当网卡名
  导致绑定失败
- 修复：main.cpp 兼容两种约定（argc>=3 用 argv[2]，argc==2 用 argv[1]）
- 结果：**无效**。实机依然不响应。

### 尝试 4：日志从 500Hz 降到 50Hz（仿照 booster_deploy_cpp，无效）

- 假设：500Hz 命令线程内做日志（6 次 `vector<double>` 分配 + DataTamer 13 字段序列化 +
  state_mutex 与 50Hz 控制线程争用）导致 low_cmd 命令间隔出现缺口，触发 Go2 底层
  看门狗，机器人停止响应
- 修复：完全仿照 booster_deploy_cpp —— 把 `LogDataStep(ms)` 从 500Hz
  `LowCmdwriteHandler` 移到 50Hz `ControlStep` 末尾；命令线程只保留纯 `Write(cmd)`；
  `t = step_count_ * ctrl->dt`；`inference_time` 传本步实测耗时
- 结果：**无效**。实机依然不响应。（x86 上无论 500Hz 还是 50Hz，`DataLogger::log()`
  实测开销都只有 ~2µs/次，命令节奏无缺口 —— 说明问题不在采样率本身）

### 尝试 5（旁路验证）：在 `dev_fix` 分支移除 DataTamer（有效）

- 把仓库回退到不含 DataTamer 的 `cfc54c9` 基线（`dev_fix` 分支）
- 结果：**实机恢复正常响应**。再次确认 DataTamer 的引入与实机不响应强相关。

## 3. 已排除的可能原因

| 假设 | 结论 |
|------|------|
| onnxruntime 符号链接/构建问题 | 排除（构建修复后依然不响应） |
| DataLogger 创建抛异常崩溃 | 排除（try/catch + create_directories 后依然不响应） |
| 命令行参数约定导致 DDS 绑错网卡 | 排除（兼容两种约定后依然不响应） |
| 500Hz 日志导致命令间隔缺口/看门狗 | 排除（降到 50Hz 后依然不响应） |
| DataTamer arm64 库与头文件 ABI 不一致 | 基本排除（arm64 静态库导出的符号与头文件 API 匹配） |
| 日志字段维度变化导致序列化崩溃 | 排除（obs/action 空→满→空切换实测不崩溃） |
| `DataLogger::log()` 单次开销过大 | 排除（x86 实测 ~2µs/次，500Hz 占 0.1% 预算） |
| 命令内容/CRC/mode/level_flag 被日志改动 | 排除（`set_cmd`/`init_low_cmd`/`Write` 与可用版本逐字节一致） |

## 4. 未验证 / 仍可疑的方向

以下方向尚未深入验证，供后续排查：

1. **DataTamer 静态库与 DDS/CycloneDDS 的链接级副作用**：`libdata_tamer.a` /
   `libmcap_lib.a` 被静态链接进同一个可执行文件，是否与 `libddsc`（CycloneDDS）或
   unitree_sdk2 存在符号/初始化顺序冲突，尚未确认。
2. **`ChannelsRegistry::Global()` 静态单例与后台写线程**：DataSinkBase 构造即启动一个
   后台线程（`try_dequeue` + `sleep_for(250us)` 循环）执行 MCAP 文件写入。该线程在
   Jetson 上的行为（CPU 占用、与 DDS 收发线程的调度竞争）未在实机测量。
3. **MCAP 文件写入位置**：实机上 `logs/` 的落盘介质（eMMC/SD/网络挂载）未知；若写入
   慢，后台写线程持续占用 CPU 或队列无限增长（moodycamel 队列无界），可能间接拖慢
   系统。需要实机监控进程 CPU/内存。
4. **Jetson 实测数据缺失**：以上所有"开销可忽略"的结论都来自 x86 测量。实机上
   `LogDataStep` 的真实耗时、命令线程实际节奏（可打时间戳统计 low_cmd 间隔）均未测。
   建议在实机加打印统计 low_cmd 发布间隔的最大值/抖动。
5. **先起 DataTamer 后起控制线程 vs 顺序对调**：当前顺序是 DataLogger 创建 →
   CreateRecurrentThreadEx(ctrl) → StartSendCmd。可尝试把 DataLogger 创建移到控制
   线程启动之后，或完全延迟到第一个 ControlStep 内再创建。

## 5. 建议的下一步排查步骤

1. 在实机上对 low_cmd 发布间隔打时间戳统计（max gap / jitter），确认命令节奏是否
   真的被破坏。若节奏正常，则"看门狗"假设不成立。
2. 用 `top`/`perf` 观察实机 CPU：DataTamer 后台写线程是否吃满一个核。
3. 做一个最小复现：在 cfc54c9（可用）上只加 DataTamer 链接 + 一个空 channel 的
   快照（不写任何 DDS 交互），看实机是否还响应。逐步二分 DataTamer 的使用面。
4. 检查 MCAP 落盘介质与写入量：50Hz × ~2KB ≈ 100KB/s，若介质极慢可能触发后台线程
   堆积。
5. 尝试 `MCAPSink(path, /*do_compression=*/false)`（当前已是 false）以及完全不建
   MCAP、只 `addDefaultSink(dummy)` 的对比实验。

## 6. 相关提交与分支

| 提交 | 说明 |
|------|------|
| `cfc54c9` | onnxruntime 控制器，**实机正常**（`dev_fix` 基线） |
| `864c776` | 引入 DataTamer 500Hz 日志，**实机不响应**（bisect 定位） |
| `d26e442` | 日志新增 PD 命令字段（dof_vel_targets/kp/kd/tau_ff） |
| `ca26d40` | 符号链接自愈 + 禁用信号处理日志冲刷 |
| `5b67832` | 接口所有权移回 RobotController + 注释 LogDataStep |
| `dev_ort` | 上述 DataTamer 系列提交所在分支 |
| `dev_fix` | 当前修复分支，基于 `cfc54c9`（实机可用） |

## 7. 结论

**DataTamer 的引入与"实机不响应"强相关（bisect 确认），但具体机制尚未定位。**
已排除：构建/符号链接、DataLogger 创建崩溃、参数约定、500Hz 采样率、命令内容被篡改。
日志单次开销在 x86 上可忽略，但**实机（Jetson）上的真实开销与线程间相互作用从未被
测量**，这是下一步排查的重点。当前以 `dev_fix`（无 DataTamer）作为实机可用基线。
