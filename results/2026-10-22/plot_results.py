#!/usr/bin/env python3
"""Regenerate the figure from the same time-aligned samples as metrics.json."""
import importlib.util
import json
from pathlib import Path
import subprocess
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

root = Path(__file__).resolve().parents[2]
output = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    'evaluation', root / 'baseline_robot_bringup/scripts/evaluate_navigation.py')
evaluation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluation)
mission = json.loads((output / 'mission.json').read_text())
start = mission['mission_start_stamp']
end = start + mission['mission_duration_sec'] + 12
streams, _ = evaluation.read_trajectory(output / 'trajectory.csv')
pairs = evaluation.aligned_errors(streams, start_time=start, end_time=end)
time = np.array([row[0] / 1e9 - start for row in pairs])
estimate = np.array([row[1] for row in pairs])
truth = np.array([row[2] for row in pairs])
error = np.array([row[3] for row in pairs])
fontfile = subprocess.check_output(
    ['fc-match', 'Noto Sans CJK SC', '-f', '%{file}'], text=True)
font_manager.fontManager.addfont(fontfile)
plt.rcParams.update({'font.family': font_manager.FontProperties(fname=fontfile).get_name(),
                     'axes.unicode_minus': False, 'font.size': 11})
fig = plt.figure(figsize=(13.5, 7.6), layout='constrained', facecolor='white')
grid = fig.add_gridspec(2, 2, width_ratios=[1.05, 1])
ax = fig.add_subplot(grid[:, 0])
waypoints = np.array([[0, 0], [0, 20], [20, 20], [20, 0], [0, 0]])
ax.plot(waypoints[:, 0], waypoints[:, 1], ':', color='#74808a', lw=1.5,
        marker='s', ms=4, label='目标航点连线')
ax.plot(truth[:, 0], truth[:, 1], color='#ed9a21', lw=3, label='Stonefish 独立真值')
ax.plot(estimate[:, 0], estimate[:, 1], '--', color='#008b65', lw=1.6,
        label='IMU / DVL / 深度 EKF')
ax.scatter(*truth[0, :2], color='#1976d2', s=45, zorder=5, label='survey 开始')
ax.scatter(*truth[-1, :2], color='#ba2d4b', marker='x', s=60, zorder=6, label='观测窗口末端')
ax.set(xlabel='东向 E / m', ylabel='北向 N / m', title='单 AUV 完整矩形航线（ENU）',
       xlim=(-2, 22), ylim=(-2, 22))
ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(loc='center', fontsize=10)
ax2 = fig.add_subplot(grid[0, 1])
ax2.axhline(3, color='#74808a', ls=':', label='目标深度 3 m')
ax2.plot(time, -truth[:, 2], color='#ed9a21', lw=2.5, label='真值')
ax2.plot(time, -estimate[:, 2], color='#008b65', ls='--', lw=1.5, label='EKF')
ax2.axvline(mission['mission_duration_sec'], color='#ba2d4b', ls=':', label='任务完成')
ax2.set(xlabel='survey 开始后时间 / s', ylabel='深度（向下为正）/ m', title='深度响应')
ax2.grid(alpha=.2); ax2.legend(fontsize=9, loc='lower right')
ax3 = fig.add_subplot(grid[1, 1])
ax3.plot(time, np.linalg.norm(error[:, :2], axis=1)*1000,
         color='#1976d2', lw=1.6, label='水平误差')
ax3.plot(time, np.linalg.norm(error, axis=1)*1000,
         color='#ba2d4b', lw=1, ls='--', label='三维误差')
ax3.axvline(mission['mission_duration_sec'], color='#ba2d4b', ls=':')
ax3.set(xlabel='survey 开始后时间 / s', ylabel='定位误差 / mm', title='时间同步后的估计与真值误差')
ax3.grid(alpha=.2); ax3.legend(fontsize=10)
fig.suptitle('2026-10-22 重新构建实测｜近零噪声、无海流｜GPS 仅作参考', fontsize=15)
fig.savefig(root / 'assets/navigation-results-baseline.png', dpi=170)
print('Saved assets/navigation-results-baseline.png')
