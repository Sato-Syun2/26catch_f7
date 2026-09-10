"""理想速度モデル時定数ごとのチャープ基本波応答を比較する。"""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('directories', type=Path, nargs='+')
    args = parser.parse_args()
    fig, axes = plt.subplots(2, 1, figsize=(9, 8), sharex=True)
    for directory in args.directories:
        result = json.loads((directory/'analysis.json').read_text())
        tau = result['model_time_constant_ms']/1000
        bands = result['frequency_bands']
        frequencies = [sum(map(float, k[:-2].split('-')))/2 for k in bands]
        line, = axes[0].plot(frequencies, [v['approx_fundamental_gain'] for v in bands.values()],
                            'o-', label=f'{tau*1000:.1f} ms measured')
        color = line.get_color()
        axes[1].plot(frequencies, [v['approx_phase_deg'] for v in bands.values()], 'o-', color=color)
        f = np.linspace(.5, 4, 150)
        axes[0].plot(f, 1/np.sqrt(1+(2*np.pi*f*tau)**2), '--', color=color, alpha=.6)
        axes[1].plot(f, -np.degrees(np.arctan(2*np.pi*f*tau)), '--', color=color, alpha=.6)
    axes[0].set_ylabel('Velocity amplitude / command amplitude')
    axes[1].set_ylabel('Phase [degrees]')
    axes[1].set_xlabel('Chirp frequency [Hz]')
    axes[0].legend()
    for ax in axes:
        ax.grid(alpha=.3)
    fig.suptitle('C620 ideal velocity model comparison\n20 mm/s, 0.5-4 Hz, 30 s; dashed = ideal model')
    fig.tight_layout()
    fig.savefig(args.output, dpi=150)


if __name__ == '__main__':
    main()
