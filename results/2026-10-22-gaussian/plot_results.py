#!/usr/bin/env python3
"""Plot the recorded acoustic fixes and EKF using the evaluated mission window."""

import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Ellipse
import numpy as np


root = Path(__file__).resolve().parents[2]
output = Path(__file__).resolve().parent
sys.path.insert(0, str(root / "baseline_robot_bringup/scripts"))
from evaluate_uncertainty import aligned_uncertainty, covariance_samples, interval_summary
spec = importlib.util.spec_from_file_location(
    "evaluation", root / "baseline_robot_bringup/scripts/evaluate_navigation.py")
evaluation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluation)
mission = json.loads((output / "mission.json").read_text())
metrics = json.loads((output / "metrics.json").read_text())
uncertainty = json.loads((output / "uncertainty.json").read_text())
start = mission["mission_start_stamp"]
end = start + mission["mission_elapsed"]
streams, _ = evaluation.read_trajectory(output / "trajectory.csv")
pairs = evaluation.aligned_errors(streams, start_time=start, end_time=end)
assert evaluation.summarize(pairs)["horizontal_rmse_m"] == metrics["horizontal_rmse_m"]
time = np.array([row[0] / 1e9 - start for row in pairs])
estimate = np.array([row[1] for row in pairs])
truth = np.array([row[2] for row in pairs])
error = np.array([row[3] for row in pairs])
acoustic = []
with (output / "trajectory.csv").open() as stream:
    for row in csv.DictReader(stream):
        stamp = int(row["stamp_sec"]) + int(row["stamp_nanosec"])*1e-9
        if row["source"] == "usbl" and start <= stamp <= end:
            acoustic.append((stamp - start, *(float(row[k]) for k in ("x", "y", "z"))))
acoustic = np.array(acoustic)
assert len(acoustic) > 100, "The complete survey needs recorded acoustic fixes"
truth_time = np.array([row[0]/1e9 - start for row in streams["truth"]])
truth_positions = np.array([row[1] for row in streams["truth"]])
acoustic_truth = np.column_stack([np.interp(acoustic[:, 0], truth_time, truth_positions[:, i])
                                  for i in range(3)])
acoustic_error = np.linalg.norm((acoustic[:, 1:] - acoustic_truth)[:, :2], axis=1)

fontfile = subprocess.check_output(["fc-match", "Noto Sans CJK SC", "-f", "%{file}"], text=True)
font_manager.fontManager.addfont(fontfile)
plt.rcParams.update({"font.family": font_manager.FontProperties(fname=fontfile).get_name(),
                     "axes.unicode_minus": False, "font.size": 10.5})
green, orange, purple, gray = "#007c5a", "#e69b28", "#9253ba", "#7e8790"
fig = plt.figure(figsize=(14, 8.7), layout="constrained", facecolor="white")
grid = fig.add_gridspec(2, 2, width_ratios=[1.07, 1])
ax = fig.add_subplot(grid[:, 0])
waypoints = np.array([[0, 0], [0, 20], [20, 20], [20, 0], [0, 0]])
ax.plot(*waypoints.T, ":", color=gray, marker="s", ms=4, lw=1.2, label="目标航点连线")
ax.plot(truth[:, 0], truth[:, 1], color=orange, lw=3.2, label="Stonefish 独立真值")
ax.scatter(acoustic[:, 1], acoustic[:, 2], s=15, facecolors="none", edgecolors=purple,
           linewidths=.8, zorder=4, label="USBL 声学观测（约 1 Hz）")
ax.plot(estimate[:, 0], estimate[:, 1], "--", color=green, lw=1.6,
        label="IMU / DVL / 深度 / USBL EKF", zorder=5)
ellipse_count, last_ellipse = 0, -float("inf")
for stamp, position, covariance in covariance_samples(output / "trajectory.csv"):
    stamp = stamp/1e9
    if not start <= stamp <= end or stamp-last_ellipse < 8:
        continue
    last_ellipse = stamp
    values, vectors = np.linalg.eigh(np.array(covariance).reshape(3, 3)[:2, :2])
    angle = np.degrees(np.arctan2(vectors[1, 1], vectors[0, 1]))
    ax.add_patch(Ellipse(position[:2], 2*np.sqrt(values[1]), 2*np.sqrt(values[0]), angle=angle,
                         facecolor=green, edgecolor=green, alpha=.22, lw=.8,
                         label="1σ 位置椭圆（每 8 s 抽样）" if ellipse_count == 0 else None))
    ellipse_count += 1
ax.scatter(*truth[0, :2], color="#276cb0", s=45, zorder=6, label="survey 开始")
ax.scatter(*truth[-1, :2], color="#bf345a", marker="x", s=65, zorder=6, label="观测窗口末端")
ax.scatter(-5, -10, color=purple, marker="*", s=180, label="固定 USBL 基站")
ax.annotate("基站 ENU = (-5, -10, -1) m", (-5, -10), xytext=(0, -9.4), fontsize=9, color=purple)
ax.set(xlabel="东向 E / m", ylabel="北向 N / m", title="全传感器非零噪声下的矩形航线（ENU）",
       xlim=(-7, 23), ylim=(-12, 23))
ax.set_aspect("equal")
ax.grid(alpha=.2)
ax.legend(loc="center", fontsize=9, framealpha=.95)

ax2 = fig.add_subplot(grid[0, 1])
ax2.axhline(3, color=gray, ls=":", label="目标深度 3 m")
ax2.scatter(acoustic[:, 0], -acoustic[:, 3], s=13, color=purple, alpha=.5,
            label="USBL z（展示，未融合）")
ax2.plot(time, -truth[:, 2], color=orange, lw=2.6, label="独立真值")
ax2.plot(time, -estimate[:, 2], color=green, ls="--", lw=1.5, label="融合估计")
ax2.axvline(mission["mission_duration_sec"], color="#bf345a", ls=":", label="任务完成")
ax2.set(xlabel="survey 开始后时间 / s", ylabel="深度（向下为正）/ m", title="深度响应：EKF 的 z 由压力计约束")
ax2.grid(alpha=.2)
ax2.legend(fontsize=8.5, loc="upper right")

ax3 = fig.add_subplot(grid[1, 1])
ax3.scatter(acoustic[:, 0], acoustic_error, s=15, facecolors="none", edgecolors=purple,
            alpha=.7, label="USBL 原始水平误差")
ax3.plot(time, np.linalg.norm(error[:, :2], axis=1), color=green, lw=1.3, label="EKF 水平误差")
ax3.axvline(mission["mission_duration_sec"], color="#bf345a", ls=":")
ax3.set(xlabel="survey 开始后时间 / s", ylabel="定位误差 / m", title="按测量时间对齐真值后的水平误差")
ax3.grid(alpha=.2)
ax3.legend(fontsize=9)
fig.suptitle("2026-10-22 高斯测量噪声实验｜实际运行数据｜独立真值仅用于评估\n"
             f"水平 RMSE {metrics['horizontal_rmse_m']:.3f} m · "
             f"末端三维误差 {metrics['final_error_3d_m']:.3f} m · "
             f"声学观测 {len(acoustic)} 点", fontsize=14)
fig.savefig(root / "assets/navigation-results.png", dpi=170)
fig.savefig(output / "navigation-results.svg")

fig3 = plt.figure(figsize=(11.5, 8), layout="constrained", facecolor="white")
space = fig3.add_subplot(111, projection="3d")
space.plot(waypoints[:, 0], waypoints[:, 1], np.full(len(waypoints), 3),
           ":", color=gray, lw=1.2, label="目标航点（深度 3 m）")
space.plot(truth[:, 0], truth[:, 1], -truth[:, 2], color=orange, lw=3,
           label="Stonefish 独立真值")
space.scatter(acoustic[:, 1], acoustic[:, 2], -acoustic[:, 3], s=13, color=purple,
              alpha=.65, label="USBL 声学位置（仅 x/y 融合）")
space.plot(estimate[:, 0], estimate[:, 1], -estimate[:, 2], "--", color=green, lw=1.7,
           label="IMU / DVL / 深度 / USBL EKF")
space.scatter(-5, -10, 1, marker="*", s=190, color=purple, label="固定 USBL 基站")
space.text(-4.5, -10, .75, "基站 (-5, -10, -1) ENU", color=purple, fontsize=9)
space.scatter(*truth[0, :2], -truth[0, 2], color="#276cb0", s=45, label="survey 开始")
space.scatter(*truth[-1, :2], -truth[-1, 2], color="#bf345a", marker="x", s=60,
              label="观测窗口末端")
space.set(xlabel="东向 E / m", ylabel="北向 N / m", zlabel="深度（向下为正）/ m",
          xlim=(-7, 23), ylim=(-12, 23), zlim=(4, -.2))
space.set_box_aspect((1, 1.16, .45))
space.view_init(elev=26, azim=-55)
space.legend(loc="upper left", fontsize=9)
space.set_title("单 AUV 双程 USBL 三维航迹｜声学观测与融合输出来自同一次运行", pad=20, fontsize=13)
fig3.savefig(root / "assets/usbl-trajectory-3d.png", dpi=170)
fig3.savefig(output / "usbl-trajectory-3d.svg")
print(f"Saved assets/navigation-results.png; {len(acoustic)} acoustic fixes in the mission window")

# Interpolate truth to filtered timestamps so each error uses its own reported P.
intervals = aligned_uncertainty(output / "trajectory.csv", start_time=start, end_time=end)
summary = interval_summary(intervals)
assert summary["coverage_2sigma"] == uncertainty["coverage_2sigma"]
interval_time = np.array([row[0]/1e9-start for row in intervals])
signed_error = np.array([row[1] for row in intervals])
sigma = np.array([row[2] for row in intervals])
fig4, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True, layout="constrained")
for i, (axis, name) in enumerate(zip(axes, ("x / 东", "y / 北", "z / 上"))):
    axis.fill_between(interval_time, -2*sigma[:, i], 2*sigma[:, i], color="#629bce", alpha=.22,
                      label="EKF 报告的 ±2σ（P 对角）")
    axis.plot(interval_time, 2*sigma[:, i], color="#629bce", lw=.6)
    axis.plot(interval_time, -2*sigma[:, i], color="#629bce", lw=.6)
    axis.axhline(0, color=gray, lw=.8)
    axis.plot(interval_time, signed_error[:, i], color=green, lw=1.1, label="估计 − 独立真值")
    axis.axvline(mission["mission_duration_sec"], color="#bf345a", ls=":", label="任务完成")
    coverage = summary["coverage_2sigma"]["xyz"[i]]
    axis.set(ylabel=f"{name} 误差 / m", title=f"{name}：实际 ±2σ 覆盖率 {coverage:.1%}")
    axis.grid(alpha=.2)
axes[0].legend(loc="upper right", ncol=3, fontsize=9)
axes[-1].set_xlabel("survey 开始后时间 / s")
fig4.suptitle(f"实际误差与报告不确定度｜{len(intervals)} 个估计时间戳\n"
              "保留原过程噪声 Q；覆盖率仅为本次统计，不代表已校准或联合 95% 区域", fontsize=13)
fig4.savefig(root / "assets/position-uncertainty.png", dpi=170)
fig4.savefig(output / "position-uncertainty.svg")

# Configured scalar noise distributions, not fitted histograms of observations.
models = uncertainty["sensor_audit"]["noise_models"]
panels = [
    ("IMU 姿态小角误差", "误差 / rad", [("roll / pitch", .005), ("yaw", .010)]),
    ("IMU 三轴角速度", "误差 / (rad/s)", [("各轴", .002)]),
    ("IMU 三轴加速度", "误差 / (m/s²)", [("各轴", .03)]),
    ("DVL 三轴速度", "误差 / (m/s)", [("各轴", models["dvl"]["velocity_stddev_m_s"])]),
    ("DVL 高度", "误差 / m", [("高度", models["dvl"]["altitude_stddev_m"])]),
    ("压力计", "误差 / Pa", [("压力", models["pressure"]["stddev_pa"])]),
    ("实体 GPS 水平位置（仅有效定位）", "误差 / m", [("水平各轴", models["gps"]["horizontal_stddev_m"])]),
    ("USBL 双程斜距", "误差 / m", [("斜距", models["usbl"]["range_stddev_m"])]),
    ("USBL 测角", "误差 / °", [("水平角", np.degrees(models["usbl"]["azimuth_stddev_rad"])),
                             ("垂直角", np.degrees(models["usbl"]["elevation_stddev_rad"]))]),
]
# Read IMU values from the run metadata too, rather than silently plotting different defaults.
panels[0][2][:] = [("roll / pitch", models["imu"]["orientation_stddev_rad"][0]),
                   ("yaw", models["imu"]["orientation_stddev_rad"][2])]
panels[1][2][:] = [("各轴", models["imu"]["angular_velocity_stddev_rad_s"][0])]
panels[2][2][:] = [("各轴", models["imu"]["linear_acceleration_stddev_m_s2"][0])]
fig5, axes5 = plt.subplots(3, 3, figsize=(14, 10), layout="constrained")
for axis, (title, xlabel, channels) in zip(axes5.flat, panels):
    scale = max(s for _, s in channels)
    x = np.linspace(-4*scale, 4*scale, 401)
    for (name, std), color in zip(channels, (green, purple)):
        pdf = np.exp(-.5*(x/std)**2)/(std*np.sqrt(2*np.pi))
        axis.plot(x, pdf, color=color, lw=1.8, label=f"{name} σ={std:.3g}")
        axis.fill_between(x, 0, pdf, color=color, alpha=.1)
    axis.axvline(0, color=gray, lw=.7, ls=":")
    axis.set(title=title, xlabel=xlabel, ylabel="概率密度（1 / 横轴单位）")
    axis.legend(fontsize=8.5)
    axis.grid(alpha=.15)
fig5.suptitle("配置模型：ε ~ N(0, σ²)，测量均值为 h(x)，误差均值为 0\n"
              "这些曲线是本次配置的模型分布，不是实测直方图；非线性变换后的协方差作一阶传播", fontsize=13)
fig5.savefig(root / "assets/sensor-noise-models.png", dpi=170)
fig5.savefig(output / "sensor-noise-models.svg")
recording = {"acoustic_fixes_in_window": len(acoustic), "uncertainty_ellipses": ellipse_count,
             "mission_start_stamp": start, "mission_end_stamp": end,
             "error_samples": len(pairs), "interval_samples": len(intervals),
             "artifacts": ["navigation-results", "usbl-trajectory-3d", "position-uncertainty", "sensor-noise-models"]}
(output / "recording.json").write_text(json.dumps(recording, indent=2) + "\n")
print(json.dumps(recording, indent=2))
plt.close("all")
