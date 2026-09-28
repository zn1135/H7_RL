# eIDE/GCC 构建环境与路径排查

本页记录 2026-09-28 在 Linux + VS Code/eIDE 上遇到的构建问题，供新克隆或换机器时排查。Keil AC5 使用另一套工具链和路径。

## 拉取代码后为什么可能无法 Build

- `.vscode/settings.json` 指定 ARM GCC 安装目录；`.eide/eide.yml` 指定 Newlib 头文件、库及 `nano.specs`、`nosys.specs`。其中 `../../.local/...` 依赖仓库与工具链恰好位于相应的目录层级，不能直接用于所有机器。
- eIDE 保存工程时可能把 `--specs` 写成当前机器的绝对路径。提交前检查 `.eide/eide.yml` 的 Git diff，避免把个人主目录路径带入团队配置。
- `EIDE.Builder.EnvironmentVariables` 中的相对 `DOTNET_ROOT` 或 `PATH` 不能假定按仓库根目录解析。这次失败时，VS Code 扩展进程从用户主目录启动，构建器找不到 `libhostfxr.so`；在终端进入仓库根目录后运行成功，不能证明 IDE 中也能运行。构建器尚未启动时，`build/` 下可能没有新日志。

## 新机器配置

1. 安装与工程兼容的 ARM GCC、Newlib 和 .NET 6 运行时。核对 `arm-none-eabi-gcc --version`、`dotnet --list-runtimes`，以及工程引用的头文件、硬浮点库、`nano.specs` 和 `nosys.specs` 是否存在。
2. 检查 `.vscode/settings.json` 与 `.eide/eide.yml` 中的 GCC 路径是否匹配本机目录。若目录层级不同，在本机调整安装布局或工程路径；不要把个人绝对路径提交到仓库。eIDE 保存或 Build 后再次检查 `git diff`，因为它可能重写工程文件。
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
| 找不到 `nano.specs`、`nosys.specs` 或 `math.h` | GCC/Newlib 是否完整安装，以及 `.eide/eide.yml` 中的头文件、库和 specs 路径。 |
| 链接报告 VFP 参数约定不一致 | `-mfloat-abi`、`-mfpu` 与 Newlib、DSP、AI 静态库的 ABI。 |

2026-09-28 本机验证：在临时输出目录完成 GCC 全量编译和链接；重载 VS Code 后，eIDE Build 也成功。此结果只证明本机当前配置可用，其他机器仍需核对上述依赖和路径。
