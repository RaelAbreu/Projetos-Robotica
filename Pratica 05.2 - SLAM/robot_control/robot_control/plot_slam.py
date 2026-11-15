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


def kidnap_time(d):
    k = d['kidnap_t']
    k = k[~np.isnan(k)]
    return float(k[0]) if k.size else None


def metrics(d):
    tk = kidnap_time(d)
    m = d['t'] < tk if tk is not None else np.ones_like(d['t'], dtype=bool)
    rms = lambda a: float(np.sqrt(np.mean(a[m] ** 2)))   # noqa: E731
    out = {
        'n': int(m.sum()),
        'slam_rmse': rms(d['err_slam_pos']), 'slam_max': float(np.max(d['err_slam_pos'][m])),
        'slam_yaw': math.degrees(rms(d['err_slam_yaw'])),
        'odom_rmse': rms(d['err_odom_pos']), 'odom_end': float(d['err_odom_pos'][m][-1]),
        'odom_yaw_end': math.degrees(float(d['err_odom_yaw'][m][-1])),
        'tk': tk,
    }
    if tk is not None:
        after = d['t'] >= tk
        ok = after & (d['err_slam_pos'] < 0.25) & (d['err_slam_yaw'] < 0.20)
        out['err_after_kidnap'] = float(d['err_slam_pos'][after][0]) if after.any() else math.nan
        out['final_err'] = float(d['err_slam_pos'][-1])
        bad = np.where(after & ~ok)[0]
        out['recovered'] = bool(after.any() and (bad.size == 0 or bad[-1] < len(d['t']) - 1 - 30))
    return out


def print_table(files, data):
    print('\nRMSE em relação ao ground truth (antes de qualquer sequestro).')
    print(f"{'arquivo':26s} {'SLAM rmse':>10} {'SLAM máx':>9} {'SLAM yaw':>9} "
          f"{'odom rmse':>10} {'odom final':>11} {'odom yaw fin.':>14}")
    print(f"{'':26s} {'[m]':>10} {'[m]':>9} {'[graus]':>9} {'[m]':>10} {'[m]':>11} {'[graus]':>14}")
    for f, d in zip(files, data):
        m = metrics(d)
        print(f"{os.path.basename(f)[:26]:26s} {m['slam_rmse']:10.3f} {m['slam_max']:9.3f} "
              f"{m['slam_yaw']:9.2f} {m['odom_rmse']:10.3f} {m['odom_end']:11.3f} "
              f"{m['odom_yaw_end']:14.1f}")
    for f, d in zip(files, data):
        m = metrics(d)
        if m['tk'] is not None:
            print(f"\n{os.path.basename(f)}: sequestro em t = {m['tk']:.1f} s; erro do SLAM logo "
                  f"depois = {m['err_after_kidnap']:.2f} m; no fim = {m['final_err']:.2f} m -> "
                  f"{'RECUPEROU' if m['recovered'] else 'NÃO recuperou'}")


def figures(files, data, plt, out_dir, show):
    names = [os.path.splitext(os.path.basename(f))[0] for f in files]
    n = len(files)

    # 1) erro de posição vs tempo
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.2))
    for name, d in zip(names, data):
        line, = axes[0].plot(d['t'], d['err_slam_pos'], label=f'{name} (SLAM)')
        axes[0].plot(d['t'], d['err_odom_pos'], '--', color=line.get_color(), alpha=.6,
                     label=f'{name} (só odometria)')
        axes[1].plot(d['t'], np.degrees(d['err_slam_yaw']), color=line.get_color(), label=name)
        tk = kidnap_time(d)
        if tk is not None:
            for ax in axes:
                ax.axvline(tk, color='r', ls=':', lw=1.5)
            axes[0].annotate('sequestro', (tk, axes[0].get_ylim()[1] * .9), color='r',
                             rotation=90, va='top', ha='right')
    axes[0].set(xlabel='t [s]', ylabel='erro de posição [m]', title='Erro de posição (vs ground truth)')
    axes[1].set(xlabel='t [s]', ylabel='erro de yaw do SLAM [graus]', title='Erro de orientação')
    for ax in axes:
        ax.grid(alpha=.3)
        ax.legend(fontsize=7)
    fig.tight_layout()
    _out(fig, plt, out_dir, 'slam_erro', show)

    # 2) trajetórias no plano
    fig, axes = plt.subplots(1, n, figsize=(5.2 * n, 5.2), squeeze=False)
    for ax, name, d in zip(axes[0], names, data):
        ax.plot(d['x_gt'], d['y_gt'], '-', color='tab:green', lw=2, label='real (ground truth)')
        ax.plot(d['x_slam'], d['y_slam'], '-', color='tab:blue', lw=1.2, label='estimada (SLAM)')
        ax.plot(d['x_odom'], d['y_odom'], '-', color='tab:orange', lw=1.2,
                label='só odometria (sem SLAM)')
        ax.set(title=name, xlabel='x [m]', ylabel='y [m]')
        ax.set_aspect('equal')
        ax.grid(alpha=.3)
        ax.legend(fontsize=7)
    fig.tight_layout()
    _out(fig, plt, out_dir, 'slam_trajetorias', show)


def map_figure(yaml_files, plt, out_dir, show):
    from PIL import Image

    n = len(yaml_files)
    fig, axes = plt.subplots(1, n, figsize=(5.2 * n, 5.2), squeeze=False)
    for ax, yf in zip(axes[0], yaml_files):
        meta = {}
        with open(os.path.expanduser(yf)) as f:
            for line in f:
                if ':' in line and not line.strip().startswith('#'):
                    k, v = line.split(':', 1)
                    meta[k.strip()] = v.strip()
        img_path = meta['image'].strip('"\'')
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.expanduser(yf)), img_path)
        res = float(meta['resolution'])
        ox, oy = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
        img = np.array(Image.open(img_path).convert('L'))
        h, w = img.shape
        ax.imshow(img, cmap='gray', origin='upper',
                  extent=[ox, ox + w * res, oy, oy + h * res])
        ax.set(title=os.path.splitext(os.path.basename(yf))[0], xlabel='x [m]', ylabel='y [m]')
    fig.tight_layout()
    _out(fig, plt, out_dir, 'slam_mapas', show)


def _out(fig, plt, out_dir, name, show):
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, f'{name}.png')
        fig.savefig(path, dpi=130)
        print(f'gravado {path}')
        if not show:
            plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('csv', nargs='+', help='CSV(s) gravados pelo slam_eval')
    ap.add_argument('--maps', nargs='+', default=[], help='.yaml dos mapas salvos')
    ap.add_argument('--summary-only', action='store_true', help='só a tabela')
    ap.add_argument('--out', default='', help='pasta para salvar PNGs')
    args = ap.parse_args()

    data = [load(f) for f in args.csv]
    print_table(args.csv, data)
    if args.summary_only:
        return
    import matplotlib
    if args.out:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    show = not args.out
    figures(args.csv, data, plt, args.out, show)
    if args.maps:
        map_figure(args.maps, plt, args.out, show)
    if show:
        plt.show()


if __name__ == '__main__':
    main()
