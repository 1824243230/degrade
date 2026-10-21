# Corridor01 回环建图与重定位验证

验证日期：2026-10-22。原始输入为
`/home/hyh/lidarbag/其他包/Corridor01.bag`，标定来自同目录的
`传感器标定参数.md`。使用用户确认的 Avia、Avia IMU 和 RGB 组合。

## 完整回环建图

使用专用脚本及经过校验的派生输入完成整段进程内回放。结果目录为
`validation/corridor01_avia_livo/runs/run_GY01Nnnu/`。

| 项目 | 结果 |
| --- | --- |
| 读取雷达消息 | 4038 |
| 读取 IMU 消息 | 82126 |
| 读取图像 | 4040 |
| 前端轨迹帧 | 4026 |
| 非有限状态更新 | 0 |
| BTC 子图 | 355 |
| 接受的回环约束 | 19 |
| 后端跳过帧 | 0 |
| 回放与后端处理时间 | 约 988 秒 |

19 条为通过校验并加入位姿图的约束数，不代表 19 个独立物理地点。
实际记录见 `loops.csv`、`graph_edges.csv` 和 `backend_summary.json`；
完整输入完成计数及实际重力、标定标识见 `full_run_summary.json`。
本次没有匹配的独立参考轨迹，因此不声称轨迹绝对精度或回环召回率。

## 地图滤波修复

原导出地图的包围盒约为 883 × 908 × 599 米。PCL 的稠密体素索引超过
32 位范围后直接返回了未降采样的点云，产生 57554718 点、约 879 MB 的地图，
实际重定位加载进程随后被系统终止。

现已将全局地图导出和定位地图滤波改为只存储已占用体素的 64 位稀疏索引，
并增加大范围、负坐标、质心和强度保持的回归测试。BTC 子图描述子生成的
原滤波流程保留；此次修复不改变已完成回放的回环约束。

基于同一完整回放的原始导出地图，使用修复后的滤波重新导出：

| 地图 | 分辨率 | 点数 | 大小 |
| --- | --- | --- | --- |
| `optimized_map_localization.pcd` | 0.2 m | 532736 | 约 8.2 MB |
| `optimized_map_localization_fine.pcd` | 0.1 m | 1846040 | 约 29 MB |

两个文件均位于上述结果目录，并保留同一坐标系、实际重力和标定元信息的
`.pcd.yaml`。原始导出文件保留，便于核查。

## 重定位验证的适用范围

已有固定地图重定位和在线回环建图是两个运行模式。专用脚本通过
`--localize MAP.pcd` 启动 KISS-Matcher 固定地图定位，并加载与建图一致的
传感器配置和标定标识。RViz 可显示已有地图，定位状态位于
`/localization/status`，只有 `valid: true` 时位姿才可用。

实际地图加载测试还发现 KISS-Matcher 对空特征数组建立 FLANN 索引时会崩溃。
已修复为返回空对应集合并继续检索其他地图块；对应的特征为空回归测试已通过，
第三方本地源码校验清单已同步更新。

受控测试中，真实 mapper 的启动定位、失效恢复及相机回放集成测试通过，
运行时间约 22 秒。该结果不是 Corridor01 的实际全局重定位成功记录。

Corridor01 包头大部分有效扫描在几米范围内。实测 0.3 m 匹配网格仅留下
106 个查询点、最多 8 对特征；0.15 m 网格约有 294 个查询点、最多 18 对特征，
未形成有效鲁棒变换。0.1 m 精细地图及室内尺度检索分块的实际搜索达到
120 秒超时，仍未进入 `TRACKING`；同目录 `localization_check_summary.json`
记录了失败状态，`initial_tracking` 和 `recovered_after_request` 均为 `false`。
当前专用配置保留可完成搜索的 0.15 m 匹配分辨率，所有匹配及几何接受门限
恢复原值。没有通过放宽门限来制造有效定位结果。
尚未获得有效定位前，不能把“节点启动、地图加载或搜索完成”视为重定位成功。

另已修复进程内定位回放的包尾等待：若最后一帧雷达需要的 IMU 时间已超过
完整 bag 的最后 IMU 时间，丢弃无法积分的尾帧并结束回放。

最终编译通过；地图滤波、回环几何、BTC 后端、重定位集成、重定位单元及状态
恢复共六项 CTest 检查全部通过，约 29 秒。定位集成检查包含新增的包尾 IMU
覆盖不足退出案例。日志保存在结果目录的 `verification_tests.log`。

## 运行命令

从任意目录直接回放并启用 BTC 与 RViz：

```bash
bash /home/hyh/salivo/scripts/run_corridor01_livo.sh use_rviz:=true
```

使用本次降采样地图尝试全局重定位：

```bash
bash /home/hyh/salivo/scripts/run_corridor01_livo.sh \
  --localize /home/hyh/salivo/validation/corridor01_avia_livo/runs/run_GY01Nnnu/optimized_map_localization.pcd \
  use_rviz:=true
```

该命令加载地图并尝试定位；是否成功需检查状态及上述验证范围。
