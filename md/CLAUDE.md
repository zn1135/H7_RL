# CLAUDE.md

> 本文件仅作指针，规则正文在 [AGENTS.md](AGENTS.md)。

接手本项目前**按顺序先读**：

1. [AGENTS.md](AGENTS.md) — 跨 AI 工作规则与约束（沟通方式、改动哲学、注释规范、参数/文档/验证纪律、红线）。
2. [RL_OVERVIEW.md](RL_OVERVIEW.md) — RL 部署总览（代码链路 + 进度 + 待实测清单 + 踩坑备忘）。
3. 其余模块文档见 `md/` 目录（DBUS / UART_IDLE_DMA / IO_CHAINS / LQR_PLAN）。

**最高约束（先看这条）**：**极性 / 零点 / 轴向正负 / MIT 量程刻度 / 符号项一律不得由 AI 改动 —— 只能由作者台架实测确定**，AI 只允许"指可疑点 + 给验证方法 + 解释现象"。详见 [AGENTS.md](AGENTS.md) **§0.1**（优先于该文件其余全部条款）。

**最高频要点**（详见 AGENTS.md）：全程中文；作者非技术背景、用产品思维理解需求、给结论别堆选项；改动最小化 + 保留运行时 A/B 开关；**参数以代码为准**（高频调参期）；使用 Keil AC5 与 `compile_commands.json` 做双配置全量编译；显示值不对**先查 Vofa 打包下标再怀疑算法**。
