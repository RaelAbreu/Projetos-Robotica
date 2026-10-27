import argparse
import csv
import math
import os

import numpy as np


def load(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f'{path}: CSV vazio')
    return {k: np.array([float(r[k]) if r.get(k, '') != '' else math.nan for r in rows])
            for k in rows[0]}


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def metrics(d, skip=0.0):
    m = d['t'] >= skip
    has_gt = not np.all(np.isnan(d['x_gt']))
    has_raw = 'x_raw' in d and not np.all(np.isnan(d['x_raw']))
    out = {'n': int(m.sum())}
    if has_gt:
        out['track_pos'] = float(np.sqrt(np.nanmean((d['x_d'][m] - d['x_gt'][m]) ** 2
                                                   + (d['y_d'][m] - d['y_gt'][m]) ** 2)))
        out['est_pos'] = float(np.sqrt(np.nanmean((d['x'][m] - d['x_gt'][m]) ** 2
                                                 + (d['y'][m] - d['y_gt'][m]) ** 2)))
        out['est_yaw_deg'] = float(np.degrees(np.sqrt(np.nanmean(
            wrap(d['th'][m] - d['th_gt'][m]) ** 2))))
        if has_raw:
            out['raw_pos'] = float(np.sqrt(np.nanmean((d['x_raw'][m] - d['x_gt'][m]) ** 2
                                                     + (d['y_raw'][m] - d['y_gt'][m]) ** 2)))
            out['raw_yaw_deg'] = float(np.degrees(np.sqrt(np.nanmean(
                wrap(d['th_raw'][m] - d['th_gt'][m]) ** 2))))
    out['w_jitter'] = float(np.std(np.diff(d['w_cmd'][m])))
    return out


def print_table(files, data, skip):
    print(f'\nResumo (amostras com t >= {skip:g} s). RMSE em relação ao ground truth.')
    hdr = (f"{'arquivo':28s} {'seguimento':>11} {'est.bruta':>10} {'est.filtro':>11} "
           f"{'yaw bruto':>10} {'yaw filtro':>11} {'osc. w_cmd':>11}")
    print(hdr)
    print(f"{'':28s} {'pos [m]':>11} {'pos [m]':>10} {'pos [m]':>11} "
          f"{'[graus]':>10} {'[graus]':>11} {'':>11}")
    for f, d in zip(files, data):
        m = metrics(d, skip)
        g = lambda k, fmt: format(m[k], fmt) if k in m else '   -'   # noqa: E731
        print(f"{os.path.basename(f)[:28]:28s} {g('track_pos', '11.3f')} {g('raw_pos', '10.3f')} "
              f"{g('est_pos', '11.3f')} {g('raw_yaw_deg', '10.2f')} {g('est_yaw_deg', '11.2f')} "
              f"{g('w_jitter', '11.3f')}")


def figures(path, d, plt, out_dir, show):
    name = os.path.splitext(os.path.basename(path))[0]
    has_raw = 'x_raw' in d and not np.all(np.isnan(d['x_raw']))
    t = d['t']

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(d['x_d'], d['y_d'], 'k--', lw=1.5, label='desejada')
    if has_raw:
        ax.plot(d['x_raw'], d['y_raw'], '.', ms=2, color='tab:orange', alpha=.5,
                label='bruta (GPS+IMU)')
    ax.plot(d['x'], d['y'], '-', lw=1.2, color='tab:blue', label='estimada (Kalman)')
    ax.plot(d['x_gt'], d['y_gt'], '-', lw=1.8, color='tab:green', label='real (ground truth)')
    ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend()
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_title(f'Trajetória - {name}')
    save(fig, out_dir, f'{name}_xy.png')

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for ax, (k, kd, kr, kg, lab) in zip(axs, [('x', 'x_d', 'x_raw', 'x_gt', 'x [m]'),
                                              ('y', 'y_d', 'y_raw', 'y_gt', 'y [m]'),
                                              ('th', 'th_d', 'th_raw', 'th_gt', 'yaw [rad]')]):
        ax.plot(t, d[kd], 'k--', label='desejada')
        if has_raw:
            ax.plot(t, d[kr], '.', ms=2, color='tab:orange', alpha=.5, label='bruta')
        ax.plot(t, d[k], color='tab:blue', label='estimada')
        ax.plot(t, d[kg], color='tab:green', label='real')
        ax.set_ylabel(lab); ax.grid(alpha=.3)
    axs[0].legend(ncol=4, loc='upper right'); axs[-1].set_xlabel('t [s]')
    fig.suptitle(f'Estados no tempo - {name}')
    save(fig, out_dir, f'{name}_estados.png')

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    if has_raw:
        axs[0].plot(t, np.hypot(d['x_raw'] - d['x_gt'], d['y_raw'] - d['y_gt']),
                    color='tab:orange', label='erro da medida bruta')
    axs[0].plot(t, np.hypot(d['x'] - d['x_gt'], d['y'] - d['y_gt']),
                color='tab:blue', label='erro da estimativa (Kalman)')
    axs[0].set_ylabel('erro de estimação [m]'); axs[0].legend(); axs[0].grid(alpha=.3)
    axs[1].plot(t, np.hypot(d['ex_gt'], d['ey_gt']), color='tab:red')
    axs[1].set_ylabel('erro de seguimento\n(real) [m]'); axs[1].grid(alpha=.3)
    axs[2].plot(t, d['v_cmd'], label='v [m/s]'); axs[2].plot(t, d['w_cmd'], label='w [rad/s]')
    axs[2].set_ylabel('comandos'); axs[2].legend(); axs[2].grid(alpha=.3)
    axs[2].set_xlabel('t [s]'); fig.suptitle(f'Erros e sinais de controle - {name}')
    save(fig, out_dir, f'{name}_erros.png')


def save(fig, out_dir, fname):
    fig.tight_layout()
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        fig.savefig(os.path.join(out_dir, fname), dpi=140)
        print('salvo:', os.path.join(out_dir, fname))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('csv', nargs='+')
    ap.add_argument('--out', default='', help='pasta para salvar os PNGs')
    ap.add_argument('--skip', type=float, default=0.0,
                    help='ignora os primeiros S segundos nas métricas (transitório inicial)')
    ap.add_argument('--summary-only', action='store_true', help='só imprime a tabela')
    args = ap.parse_args()

    data = [load(f) for f in args.csv]
    print_table(args.csv, data, args.skip)
    if args.summary_only:
        return
    import matplotlib
    if args.out:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for f, d in zip(args.csv, data):
        figures(f, d, plt, args.out, show=not args.out)
    if not args.out:
        plt.show()
