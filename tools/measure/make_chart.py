"""README chart: frame-to-frame shake of the rendered output, measured by re-tracking the renders
(tools/measure/panel_measure.py). Shake = image motion of the middle third of the frame minus its
0.5 s moving average (i.e. everything faster than ~1 Hz), px per frame at 1080 px output width,
exactly as seen on screen (no correction for the different zoom).
usage: run in a folder with m_<clip>_<panelX>.npz from panel_measure.py (panelX 0 / 1080 / 2160) -> shake_comparison.png"""
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from scipy.ndimage import uniform_filter1d
for f in ['/usr/share/fonts/opentype/inter/Inter-Regular.otf', '/usr/share/fonts/opentype/inter/Inter-SemiBold.otf']:
    fm.fontManager.addfont(f)
plt.rcParams.update({'font.family': 'Inter', 'font.size': 11, 'axes.edgecolor': '#c3c2b7', 'axes.labelcolor': '#52514e',
                     'xtick.color': '#52514e', 'ytick.color': '#52514e', 'text.color': '#0b0b0b'})
S = [('Original (Action Mode)', 0, '#1baf7a'), ('Resolve Stabilizer', 1080, '#eb6834'), ('Trail Stability', 2160, '#2a78d6')]
def shake(c, x, zone=1):
    m = np.load(f'm_{c}_{x}.npz')['m']; d = m[:, zone * 3:zone * 3 + 2].copy()
    for k in range(2):
        v = d[:, k]; bad = np.isnan(v)
        if bad.any(): v[bad] = np.interp(np.flatnonzero(bad), np.flatnonzero(~bad), v[~bad])
    hp = d - uniform_filter1d(d, 30, axis=0, mode='nearest')
    return np.hypot(hp[:, 0], hp[:, 1])
fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.2), gridspec_kw={'width_ratios': [2.3, 1]}, dpi=150)
fig.patch.set_facecolor('#fcfcfb')
for ax in (a1, a2):
    ax.set_facecolor('#fcfcfb'); ax.grid(axis='y', color='#e8e7e2', lw=0.8); ax.set_axisbelow(True)
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
# left: shake over time, Clip 3
for name, x, col in S:
    s = shake(1854, x); r = np.sqrt(uniform_filter1d(s ** 2, 60))
    t = np.arange(len(r)) / 60.0
    a1.plot(t, r, color=col, lw=2, label=name)
    a1.text(t[-1] + 0.3, r[-30:].mean(), name.split(' (')[0], color='#52514e', va='center', fontsize=10)
a1.set_xlim(0, t[-1] + 5.5); a1.set_ylim(0, None)
a1.set_xlabel('time (s)'); a1.set_ylabel('shake (px per frame, 1 s RMS)')
a1.set_title('Shake over time - Clip 3 (23.6 s trail run)', loc='left', fontsize=12, fontweight='semibold')
# right: mean shake per clip
clips = [1852, 1853, 1854]; w = 0.26
for i, (name, x, col) in enumerate(S):
    vals = [np.sqrt(np.mean(shake(c, x) ** 2)) for c in clips]
    xs = np.arange(len(clips)) + (i - 1) * (w + 0.02)
    a2.bar(xs, vals, w, color=col, label=name)
    for xx, v in zip(xs, vals): a2.text(xx, v + 0.08, f'{v:.1f}', ha='center', fontsize=9, color='#52514e')
a2.set_xticks(range(len(clips)), [f'Clip {i + 1}' for i in range(len(clips))]); a2.set_ylabel('shake (px per frame, RMS)')
a2.set_title('Average per clip', loc='left', fontsize=12, fontweight='semibold')
h, l = a2.get_legend_handles_labels()
fig.legend(h, l, loc='lower center', ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.02))
fig.tight_layout(rect=(0, 0.07, 1, 1))
fig.savefig('shake_comparison.png', facecolor=fig.get_facecolor())
for c in clips:
    v = [np.sqrt(np.mean(shake(c, x) ** 2)) for _, x, _ in S]
    print(c, ' '.join(f'{x:.2f}' for x in v), f'trail vs orig {100*(v[2]/v[0]-1):.0f}%  resolve vs orig {100*(v[1]/v[0]-1):.0f}%')
