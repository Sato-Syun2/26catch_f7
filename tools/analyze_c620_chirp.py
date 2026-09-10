"""C620チャープの方向別追従・電流・移動量を集計。電流はトルクの代用値。"""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def analyze(directory):
    meta = json.loads((directory / 'metadata.json').read_text())
    data = np.genfromtxt(directory / 'feedback.csv', delimiter=',', names=True,
                         dtype=None, encoding='utf-8')
    active = ((data['state'] != 0) & (data['time'] >= meta.get('drive_start', 0)) &
              (data['time'] <= meta.get('drive_end', float('inf'))))
    d = data[active]
    if not len(d):
        raise ValueError('no active samples')
    # 約100Hzの量子化速度を100ms平均。端のゼロ詰めは集計から除外。
    velocity = np.convolve(d['velocity'], np.ones(10)/10, mode='same')
    valid = np.ones(len(d), dtype=bool)
    valid[:5] = valid[-5:] = False
    # 100Hzログから一次遅れを再構成。500Hz内部状態の実測値ではない。
    model = np.zeros(len(d))
    model[0] = d['velocity'][0]
    for i in range(1, len(d)):
        beta = np.exp(-meta.get('alpha', 5)*(d['time'][i]-d['time'][i-1]))
        model[i] = beta*model[i-1]+(1-beta)*d['target'][i-1]
    summary = {'result': meta['result'], 'disabled': meta.get('disabled'),
               'model_time_constant_ms': 1000.0/meta.get('alpha', 5),
               'position_min': float(d['position'].min()),
               'position_max': float(d['position'].max()),
               'net_displacement': float(d['position'][-1]-d['position'][0]),
               'commanded_displacement': float(np.trapz(d['target'], d['time'])),
               'peak_abs_velocity_raw': float(np.max(np.abs(d['velocity']))),
               'peak_abs_velocity_100ms': float(np.max(np.abs(velocity[valid]))),
               'peak_abs_current': float(np.max(np.abs(d['current']))),
               'rms_tracking_error_100ms': float(np.sqrt(np.mean(
                   (velocity[valid]-d['target'][valid])**2)))}
    model_filtered = np.convolve(model, np.ones(10)/10, mode='same')
    summary['rms_model_error_100ms'] = float(np.sqrt(np.mean(
        (velocity[valid]-model_filtered[valid])**2)))
    summary['rms_model_error_raw'] = float(np.sqrt(np.mean(
        (d['velocity'][valid]-model[valid])**2)))
    summary['displacement_error'] = summary['net_displacement']-summary['commanded_displacement']
    if not meta.get('position_only', False):
        elapsed = d['time']-meta['drive_start']
        frequency = meta['f0']+(meta['f1']-meta['f0'])*elapsed/meta['duration']
        phase = 2*np.pi*(meta['f0']*elapsed +
                        (meta['f1']-meta['f0'])*elapsed**2/(2*meta['duration']))
        summary['frequency_bands'] = {}
        for lo, hi in [(.5, 1), (1, 2), (2, 3), (3, 4)]:
            selected = valid & (frequency >= lo) & (frequency < hi)
            if selected.sum() < 20:
                continue
            basis = np.column_stack([np.sin(phase[selected]), np.cos(phase[selected]),
                                     np.ones(selected.sum())])
            u = np.linalg.lstsq(basis, d['target'][selected], rcond=None)[0]
            v = np.linalg.lstsq(basis, d['velocity'][selected], rcond=None)[0]
            gain = np.hypot(v[0], v[1])/np.hypot(u[0], u[1])
            phase_delta = np.angle(complex(v[0],v[1])/complex(u[0],u[1]), deg=True)
            summary['frequency_bands'][f'{lo}-{hi}Hz'] = {
                'approx_fundamental_gain': float(gain),
                'approx_phase_deg': float(phase_delta)}
    for direction, sign in [('extension', 1), ('retraction', -1)]:
        select = valid & (sign*d['target'] > .5)
        moving = select & (sign*velocity > .5)
        summary[direction] = {
            'samples': int(select.sum()),
            'mean_target': float(d['target'][select].mean()) if select.any() else None,
            'mean_velocity_100ms': float(velocity[select].mean()) if select.any() else None,
            'rms_error_100ms': float(np.sqrt(np.mean(
                (velocity[select]-d['target'][select])**2))) if select.any() else None,
            'median_abs_current_while_moving': float(np.median(
                np.abs(d['current'][moving]))) if moving.any() else None,
            'moving_fraction': float(moving.sum()/select.sum()) if select.any() else None}
        # 指令反転時の遅れを混ぜないよう、実速度の符号でも分類する。
        matched_speed = valid & (sign*velocity >= 2) & (sign*velocity <= 10)
        summary[direction]['median_abs_current_at_2_to_10mm_s'] = float(np.median(
            np.abs(d['current'][matched_speed]))) if matched_speed.any() else None
        summary[direction]['matched_speed_samples'] = int(matched_speed.sum())
    (directory/'analysis.json').write_text(json.dumps(summary, indent=2))
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    t = d['time']-d['time'][0]
    axes[0].plot(t, d['target'], label='Velocity command')
    axes[0].plot(t, model, '--', label='Reconstructed ideal model')
    axes[0].plot(t, d['velocity'], alpha=.25, label='Raw feedback')
    axes[0].plot(t, velocity, label='100ms mean')
    axes[0].set_ylabel('Velocity [configured mm/s]')
    axes[0].legend()
    axes[1].plot(t, d['position'])
    axes[1].set_ylabel('Position [configured mm]')
    axes[2].plot(t, d['current'])
    axes[2].set_ylabel('CAN current [A]')
    axes[2].set_xlabel('Time [s]')
    for ax in axes:
        ax.grid(alpha=.3)
    fig.suptitle(directory.name + '\n' +
                 f"Model time constant={1000.0/meta.get('alpha',5):.1f} ms, " +
                 f"DOB={meta.get('bandwidth')} rad/s, kp={meta.get('kp')}, ki={meta.get('ki')}")
    fig.tight_layout()
    fig.savefig(directory/'response.png', dpi=140)
    plt.close(fig)
    print(directory.name, json.dumps(summary))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directories', type=Path, nargs='+')
    for directory in parser.parse_args().directories:
        analyze(directory)
