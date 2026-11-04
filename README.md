# RoboMaster 底盘：标准库到 HAL 的重构

从既有麦轮机器人底盘移植到 HAL，将控制任务、遥控、裁判、离线检测与功率控制重新组织，并保留综合移植与后续精简两个阶段。

**当前分支**：`archive/early-hal-integration`。7 月 3 日前后的综合 HAL 移植树。

**留存状态**：底盘重构开发中。

[查看项目时间线](docs/TIMELINE.md)

## 来源

Baby 标准库底盘与 RoboMaster 相关模块；沿用代码来源和个人迁移工作待分别署名。

个人职责待补充。保留原工程中的来源说明和许可证；本次归档没有进行构建、烧录或实机验收。

## 代码与历史分支

| 分支 | 内容 |
| --- | --- |
| `main` | 项目代表工作树，包含本次保存的现存源码。 |
| [`archive/early-hal-integration`](https://github.com/RynStryn/rm-chassis-hal/tree/archive/early-hal-integration) | 7 月 3 日前后的综合 HAL 移植树 |
| [`archive/baby-standard-library`](https://github.com/RynStryn/rm-chassis-hal/tree/archive/baby-standard-library) | Baby 标准库整车来源工作树 |

## 时间口径

历史研发／修改日期写在时间线中。本次于 2026-11-04 整理并提交现存源码；新快照提交使用实际提交日期。

有原 Git 历史的工作树以原 HEAD 为父提交保留历史；无 Git 历史的树以独立根提交保存，不补造历史提交。工作区中的源码变动一并保存在本次快照。

本次快照排除了构建输出、IDE 会话文件和通用整理资料。历史提交保持原样，其中可能保留以前已跟踪的工具状态或构建文件。
