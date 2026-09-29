# eIDE/GCC 构建环境与路径排查

本页记录 2026-09-28 在 Linux + VS Code/eIDE 上遇到的构建问题，供新克隆或换机器时排查。Keil AC5 使用另一套工具链和路径。

## 拉取代码后为什么可能无法 Build

- ARM GCC 安装目录由本机 eIDE 设置提供；`.eide/eide.yml` 使用工具链自带的 Newlib 头文件、库、`nano.specs` 和 `nosys.specs`。不要将 `../../.local/...` 这类依赖目录层级的路径写进工程。
- eIDE 保存工程时可能把 `--specs` 写成当前机器的绝对路径。提交前检查 `.eide/eide.yml` 的 Git diff，避免把个人主目录路径带入团队配置。
- `EIDE.Builder.EnvironmentVariables` 中的相对 `DOTNET_ROOT` 或 `PATH` 不能假定按仓库根目录解析。这次失败时，VS Code 扩展进程从用户主目录启动，构建器找不到 `libhostfxr.so`；在终端进入仓库根目录后运行成功，不能证明 IDE 中也能运行。构建器尚未启动时，`build/` 下可能没有新日志。

## 新机器配置

1. 安装与工程兼容的 ARM GCC、Newlib 和 .NET 6 运行时。核对 `arm-none-eabi-gcc --version`、`dotnet --list-runtimes`，以及工程引用的头文件、硬浮点库、`nano.specs` 和 `nosys.specs` 是否存在。
2. 在本机 VS Code **用户设置**中配置 `EIDE.ARM.GCC.InstallDirectory` 为 ARM GCC 安装目录（例如 Windows 的 `D:\Cubeclt\STM32CubeCLT_1.18.0\GNU-tools-for-STM32`）。核对 `.eide/eide.yml` 中的 `--specs=nano.specs --specs=nosys.specs`，不要把个人绝对路径提交到仓库。eIDE 保存或 Build 后再次检查 `git diff`，因为它可能重写工程文件。
3. 在本机 VS Code **用户设置**中为 `EIDE.Builder.EnvironmentVariables` 配置实际存在的绝对路径。以下 `/实际用户目录` 只是示例，必须替换；`PATH` 应保留系统命令目录。用户设置不随 Git 克隆同步。若工作区设置中也定义了同名项，应先检查它是否覆盖用户设置。

   ```json
   {
     "EIDE.Builder.EnvironmentVariables": [
       "DOTNET_ROOT=/实际用户目录/.local/share/dotnet",
       "PATH=/实际用户目录/.local/bin:/实际用户目录/.local/share/dotnet:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
     ]
   }
   ```

4. 执行 VS Code 命令 **Developer: Reload Window**，再点 eIDE **Build**。换工具链位置后，还要核对 GCC 架构、浮点 ABI 和 Newlib 库目录。

## 按报错定位

| 现象 | 先检查 |
| --- | --- |
| `libhostfxr.so` 缺失，且没有新编译日志 | .NET 运行时、`DOTNET_ROOT` 和 VS Code 扩展进程使用的路径。 |
| 找不到 `nano.specs`、`nosys.specs` 或 `math.h` | 本机 eIDE 使用的 GCC 安装目录、GCC/Newlib 是否完整，以及 `.eide/eide.yml` 是否引用了旧机器的路径。 |
| 链接报告 VFP 参数约定不一致 | `-mfloat-abi`、`-mfpu` 与 Newlib、DSP、AI 静态库的 ABI。 |

2026-09-28 本机验证：在临时输出目录完成 GCC 全量编译和链接；重载 VS Code 后，eIDE Build 也成功。此结果只证明本机当前配置可用，其他机器仍需核对上述依赖和路径。

## GCC 编译通过但 VOFA 停止、控制不运行

2026-09-28 使用 GCC 13.3.1、`-O0` 的固件出现：彩灯持续变化，VOFA 只发出启动时的一组策略帧，此后策略序号不再增加。TIM6 仍在运行，但策略和输出任务阻塞在节拍信号量上。

板端硬件写监视确认，`defaultTask` 执行 USB 初始化时栈溢出，覆盖了先分配的两个信号量。信号量创建后容量为 1，随后被 `HAL_PCD_MspInit()` 调用的 `HAL_RCCEx_PeriphCLKConfig()` 的栈写入清零；TIM6 中断中 CPU 实际读取到容量 0，释放信号量失败。不能将此现象归因于 VOFA 波特率或模型推理。

原任务栈为 128 个 32 位字，即 512 字节；该 GCC 构建中，`HAL_PCD_MspInit()` 和嵌套的 `HAL_RCCEx_PeriphCLKConfig()` 栈帧分别为 208 和 312 字节，两层已超过原容量，尚未计入上层调用和上下文保存。修复为 512 个字（2048 字节），同时修改 `.ioc` 的 `FREERTOS.Tasks01` 与 `Core/Src/freertos.c` 的对应生成行，避免 CubeMX 再生成时回退。AC5 下可运行不代表相同任务栈也适用于 GCC。

重新生成源码指纹并构建后，在失能状态烧录验收：检查 VOFA 策略序号持续增加、时间戳递增，解码后无丢样；如连接调试器，核对两个信号量容量保持 1，并检查 USB 初始化后的任务栈剩余量。编译和链接通过不能替代这一步。若随后仍不能使能，按新帧中的故障位继续检查 IMU、遥控、电机与 CAN。
