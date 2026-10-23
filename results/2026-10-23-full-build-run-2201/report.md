# 2026-10-23 完整编译与运行验证

已完成当前工作区代码的构建、测试及默认 PF＋USBL 完整矩形航线。

- 依赖：11 个包构建成功，复用已有锁定版本源码和构建缓存。
- 本项目：从新生成的 build/install 构建 4 个包，启用 BUILD_TESTING。
- 测试：74 项 Python 测试全部通过；colcon 含 6 个 CTest 包装项统计为 80 项，零失败、零错误、零跳过。
- 运行：Stonefish、RViz、PF、传感器适配、控制器和任务节点实际启动；隔离 ROS_DOMAIN_ID=181。
- 航线：20×20 m 矩形的全部航点完成，126.88 秒后返回 start，再观测 12 秒后关闭控制器。
- PF 输出：3037 个实际订阅样本，平均 19.93 Hz。轨迹 CSV 按 sample_period=0.2 秒降采样保存，CSV 行频率不代表 PF 发布频率。
- 水平 RMSE：0.044001 m。
- 三维 RMSE：0.047225 m。
- 最大三维误差：0.115914 m。
- 末端三维误差：0.048797 m。
- PF 三项阈值分别为 0.30/0.35/0.35 m，本次全部通过。指标覆盖预热 5 秒后的记录窗口，包含任务前后观测。
- 传感器协方差检查：通过，零协方差不匹配，无缺失数据源。水下实体 GPS 的 NO_FIX 及启动时少量无效压力测量已记录，不参与有效观测。
- 退出：15 个 launch 子进程正常退出，RViz 退出码 0；未留下本次仿真进程。

Stonefish 仍有两条 `Failed to link program!` 海面着色器报错，与 README 已记载的问题一致；本次未阻止仿真及航线完成。

保留全部已有源码修改；本次仅生成构建产物、恢复 `.deps` 缓存链接并新增本结果目录。

## 原始记录

- [依赖构建](dependencies-build.log)、[项目构建](build.log)
- [测试详细日志](test.log)、[colcon 测试汇总](test-result.log)
- [运行输出](validation.log)、[仿真与退出日志](survey/launch.log)、[RViz 日志](rviz.log)
- [任务状态及 PF 诊断](survey/mission.json)、[定位指标](survey/metrics.json)
- [传感器与协方差验收](survey/uncertainty.json)
- [轨迹 CSV](survey/trajectory.csv)、[传感器原始记录](survey/trajectory-sensors.jsonl)
- [节点](nodes.txt)、[话题](topics.txt)、[PF 订阅与发布](pf-node.txt)
- [机器可读汇总](summary.json)、[本次执行脚本](run_full_validation.py)
