# 问题记录：引入 DataTamer 数据日志后实机不响应（已定位根因）

> 状态：**已定位根因（onnxruntime CPUIDInfo 越界 bug），已升级修复**。
> 根因**不是 DataTamer**，而是旧的 onnxruntime 1.23.2 在 Jetson（arm64）上
> `CPUIDInfo::ArmLinuxInit()` 存在 heap 越界写，导致堆损坏与偶发崩溃。已升级到
> 官方 onnxruntime 1.27.1（含上游修复 PR #28344）。本文记录问题现象、定位过程、
> 已尝试的修复、最终根因与处理结果。

## 1. 问题现象

在 Unitree Go2 实机（Jetson Orin NX，arm64）上运行带 DataTamer 数据日志的控制器：

- ✅ 能正常收到机器人状态反馈（`rt/lowstate`）与遥控器数据
- ✅ 程序正常发布 low_cmd（`rt/lowcmd`），发布链路本身工作
- ❌ **机器人不执行任何命令**（不站立、不运动、无响应）

在 Jetson 上运行时，旧 onnxruntime 1.23.2 还会在**启动阶段**直接崩溃：

```
onnxruntime cpuid_info warning: Unknown CPU vendor. cpuinfo_vendor value: 0
/opt/rh/gcc-toolset-14/root/usr/include/c++/14/bits/stl_vector.h:1130:
  std::vector<unsigned int>::operator[] ... Assertion '__n < this->size()' failed.
Aborted (core dumped)
```

通过 git bisect 定位到首个"坏"提交：

```
864c776 [feat]：新增 DataTamer 高频数据日志（500Hz MCAP），移除旧 CSV 日志
  ↑ 引入 DataTamer 后实机不响应
cfc54c9 [feat]：切换至 onnxruntime 推理，统一为 metadata 驱动的 Go2OnnxController
  ↑ 控制器正常，实机响应正常
```

当初的结论是"onnxruntime 推理本身没问题（cfc54c9 正常），一旦叠加 DataTamer 日志就
实机不响应"——**该结论是误导性的**。真正的问题是 onnxruntime 1.23.2 的 CPUIDInfo
越界 bug（见第 6 节），叠加 DataTamer 只是改变了堆布局，让同一处堆损坏的**表现**不同
（不响应 vs 崩溃），DataTamer 本身不是根因。

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

## 4. 最终根因：onnxruntime CPUIDInfo 越界 bug（与 DataTamer 无关）

在 Jetson 上运行旧 onnxruntime 1.23.2 时，启动阶段出现：

```
onnxruntime cpuid_info warning: Unknown CPU vendor. cpuinfo_vendor value: 0
stl_vector.h:1130: std::vector<unsigned int>::operator[] Assertion '__n < this->size()' failed.
Aborted (core dumped)
```

### 4.1 根因定位

- 崩溃类型是 `std::vector<unsigned int>::operator[]` 越界断言，发生在 onnxruntime
  会话初始化期间（`cpuid_info warning` 打印后立即崩溃）。
- 仓库内 `third_party/onnxruntime/lib/arm64/libonnxruntime.so.1.23.2` 由
  gcc-toolset-14 构建并启用了 `_GLIBCXX_ASSERTIONS`（lib 内嵌 `/opt/rh/gcc-toolset-14/
  .../stl_vector.h` 与 `__n < this->size()` 断言串）。因此 operator[] 越界会**直接
  abort**，而不是静默写坏堆。
- 越界代码在 onnxruntime 1.23.2 `core/common/cpuid_info.cc` 的
  `CPUIDInfo::ArmLinuxInit()`：

  ```cpp
  const uint32_t core_cnt = cpuinfo_get_cores_count();
  core_uarchs_.resize(core_cnt, cpuinfo_uarch_unknown);   // vector<uint32_t> = vector<unsigned int>
  ...
  auto coreid = proc->linux_id;                            // Linux 逻辑 CPU 号
  core_uarchs_[coreid] = uarch;                            // 若 linux_id >= core_cnt 则越界写
  ```

  在 Jetson Orin 上 `cpuinfo` 对 CPU 拓扑的解析可能报告稀疏/非标准 `linux_id`
  （Unknown CPU vendor），导致 `linux_id >= core_cnt` → `core_uarchs_[coreid]` 越界写。

### 4.2 上游修复

- 这是 onnxruntime 官方已知 bug：GitHub issue **#28301**（"CPUIDInfo crashes with
  free(): invalid pointer on ARM (Jetson Orin) when CPU vendor is unknown"，Jetson
  Orin NX 100% 复现）。
- 修复 PR **#28344**（2026-06-02 合并）：在 `ArmLinuxInit()` 使用 `linux_id` 索引前加
  边界检查：

  ```cpp
  auto coreid = proc->linux_id;
  if (coreid < 0 || static_cast<size_t>(coreid) >= core_uarchs_.size()) {
    continue;
  }
  ```

- 修复随 **onnxruntime v1.27.0**（2026-06-19 发布）起进入正式版本。**v1.24.4 及更早
  均未包含此修复**（v1.24.4 发布于 2026-03-17，早于修复合并）。

### 4.3 为什么之前"时好时坏"/"看起来像 DataTamer 的问题"

- 未启用断言（Release 常规构建）时，`core_uarchs_[coreid]` 越界写是**静默堆损坏**，
  不立即崩溃。
- 是否崩溃 / 表现为何种故障，取决于越界写恰好破坏了哪块相邻堆内存。引入 DataTamer
  改变了堆分配布局 → 同一处越界写的**表现**改变（"不响应" vs 直接崩溃），造成
  "DataTamer 导致实机不响应"的假象。
- 参考项目 booster_deploy_cpp 同样 vendored 了同一个有 bug 的 1.23.2 arm64 lib，
  只是尚未触发该 CPU 拓扑条件，并非"50Hz 日志没问题"。

## 5. 处理结果：升级 onnxruntime 到 1.27.1

1. 下载官方 **onnxruntime-linux-aarch64-1.27.1.tgz** 与
   onnxruntime-linux-x64-1.27.1.tgz（含 #28344 修复）。
2. 替换 `third_party/onnxruntime/lib/{arm64,x86_64}/` 下的
   `libonnxruntime.so*` 为 1.27.1（旧 1.23.2 备份在 `lib/<arch>/1.23.2_backup/`）。
3. 替换 `third_party/onnxruntime/include/` 头文件为 1.27.1（1.23.2 头文件备份在
   `/tmp/ort_dl/headers_1.23.2_backup/`）。
4. **CMakeLists.txt 头文件路径调整**：1.27.1 头文件从 `include/core/session/` 移到了
   `include/`，`include_directories` 由 `${ONNXRUNTIME_DIR}/include/core/session`
   改为 `${ONNXRUNTIME_DIR}/include`。
5. 清理重建（`rm -rf build && cmake .. && make`），x86_64 验证：
   - 编译链接通过
   - 策略加载正常（`policy loaded ... obs_dim=135`）
   - 会话创建无崩溃、无 `cpuid_info warning`
6. **待实机验证**：在 Jetson 上重新部署 arm64 二进制，确认：
   - 启动不再崩溃（1.27.1 含边界检查）
   - 机器人正常响应（若此前"不响应"也是同一堆损坏导致，则一并解决）
   - 重新叠加 DataTamer 日志（50Hz），确认日志功能可正常工作

> API 兼容性：项目只用到稳定公共 API（`Ort::Env` / `Ort::Session` / `Run` /
> `CreateTensor` / `MemoryInfo` / `GetInputNameAllocated` / `GetOutputNameAllocated` /
> `GetCustomMetadataMap`），1.23 → 1.27.1 全部存在且签名一致，无需改业务代码。

## 6. 相关提交与分支

| 提交 | 说明 |
|------|------|
| `cfc54c9` | onnxruntime 控制器（vendored 1.23.2，含 CPUIDInfo bug） |
| `864c776` | 引入 DataTamer 500Hz 日志（后被误判为根因） |
| `d26e442` | 日志新增 PD 命令字段（dof_vel_targets/kp/kd/tau_ff） |
| `ca26d40` | 符号链接自愈 + 禁用信号处理日志冲刷 |
| `5b67832` | 接口所有权移回 RobotController + 注释 LogDataStep |
| `dev_ort` | 当前分支（DataTamer 系列 + 问题文档 + onnxruntime 1.27.1 升级） |

## 7. 结论

**根因是 onnxruntime 1.23.2 在 Jetson（arm64）上的 `CPUIDInfo::ArmLinuxInit()`
越界写 bug（issue #28301 / PR #28344），与 DataTamer 无关。** 旧 lib 以
`_GLIBCXX_ASSERTIONS` 构建时在启动阶段直接 abort；常规构建则为静默堆损坏，其表现
（"不响应" vs 崩溃）随堆布局变化而变，造成"DataTamer 导致实机不响应"的假象。

**已修复**：升级 onnxruntime 到 1.27.1（官方，含上游边界检查），x86 构建验证通过，
DataTamer 相关业务代码无需改动。**剩余工作**：在 Jetson 实机重新部署验证，并恢复
DataTamer 日志（50Hz）复测。
