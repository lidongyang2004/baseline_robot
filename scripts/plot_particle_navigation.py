#!/usr/bin/env python3
"""Plot two independent PF survey runs; no alignment or ground-truth feedback."""
import argparse
import os
from pathlib import Path
import sys

os.environ.setdefault('MPLCONFIGDIR', '/tmp/baseline-pf-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'baseline_robot_bringup/scripts'))
from evaluate_navigation import read_trajectory, aligned_errors, summarize


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('with_usbl', type=Path)
    parser.add_argument('without_usbl', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists')
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for column, (folder, title) in enumerate([(args.with_usbl, 'PF with USBL'),
                                             (args.without_usbl, 'PF without USBL')]):
        streams, _ = read_trajectory(folder/'trajectory.csv')
        pairs = aligned_errors(streams)
        metrics = summarize(pairs)
        estimates = np.array([row[1] for row in pairs])
        truth = np.array([row[2] for row in pairs])
        times = np.array([(row[0]-pairs[0][0])*1e-9 for row in pairs])
        error = np.array([row[3] for row in pairs])
        ax = axes[0, column]
        ax.plot(truth[:, 0], truth[:, 1], color='#eb8c33', lw=2, label='Independent truth')
        ax.plot(estimates[:, 0], estimates[:, 1], color='#137f65', lw=1.1, label='Particle filter')
        ax.set(title=title, xlabel='East (m)', ylabel='North (m)')
        ax.axis('equal'); ax.grid(alpha=.25); ax.legend(loc='lower right', fontsize=8)
        ax.text(.03, .96, f"Horizontal RMSE: {metrics['horizontal_rmse_m']:.3f} m\n"
                f"Max 3D error: {metrics['max_error_3d_m']:.3f} m", transform=ax.transAxes,
                va='top', fontsize=9, bbox=dict(facecolor='white', edgecolor='none', alpha=.85))
        ax = axes[1, column]
        ax.plot(times, np.linalg.norm(error[:, :2], axis=1), label='Horizontal error', color='#137f65')
        ax.plot(times, np.linalg.norm(error, axis=1), label='3D error', alpha=.7, color='#3779aa')
        ax.set(xlabel='Time after evaluation warmup (s)', ylabel='Position error (m)', ylim=(0, .25))
        ax.grid(alpha=.25); ax.legend(fontsize=8)
    figure.suptitle('Paper-structure planar particle navigation | separate Stonefish runs', fontsize=13)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=160)
    plt.close(figure)


if __name__ == '__main__':
    main()
