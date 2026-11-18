"""Bewegung des Original-Panels (links) eines 3-Panel-Renders, nur texturierte Gitterpunkte, pro Drittel.
usage: panel_measure.py render.mp4 out.npz   -> m[k] = [x,y,n]*4 (oben, mitte, unten, gesamt), px bei 1080 Breite"""
import numpy as np, cv2, subprocess, sys
F, out = sys.argv[1], sys.argv[2]
X0 = int(sys.argv[3]) if len(sys.argv) > 3 else 0   # 0 = Original, 2160 = TrailStab-Panel
W, H = 3240, 1920; WM, HM = 540, 960
p = subprocess.Popen(['ffmpeg', '-v', 'error', '-i', F, '-vf', f'crop=1080:1920:{X0}:0,scale=540:960:flags=area,format=gray',
                      '-f', 'rawvideo', '-'], stdout=subprocess.PIPE, bufsize=1 << 24)
step = 12
gy, gx = np.mgrid[step:HM - step:step, step:WM - step:step]
grid = np.column_stack([gx.ravel(), gy.ravel()]).astype(np.float32).reshape(-1, 1, 2)
yy = grid.reshape(-1, 2)[:, 1]
zones = [(0, HM / 3), (HM / 3, 2 * HM / 3), (2 * HM / 3, HM), (0, HM)]
res = []; prev = None
while True:
    b = p.stdout.read(WM * HM)
    if len(b) < WM * HM: break
    g = np.frombuffer(b, np.uint8).reshape(HM, WM)
    if prev is not None:
        ev = cv2.cornerMinEigenVal(prev, 7)
        tex = ev[grid[:, 0, 1].astype(int), grid[:, 0, 0].astype(int)] > 1e-3
        q, st, _ = cv2.calcOpticalFlowPyrLK(prev, g, grid, None, winSize=(21, 21), maxLevel=3)
        qb, st2, _ = cv2.calcOpticalFlowPyrLK(g, prev, q, None, winSize=(21, 21), maxLevel=3)
        ok = tex & (st.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm((qb - grid).reshape(-1, 2), axis=1) < 0.3)
        d = (q - grid).reshape(-1, 2) * 2
        row = []
        for z0, z1 in zones:
            m = ok & (yy >= z0) & (yy < z1)
            row += list(np.median(d[m], 0)) + [m.sum()] if m.sum() > 15 else [np.nan, np.nan, m.sum()]
        res.append(row)
    prev = g
np.savez(out, m=np.array(res)); print(out, len(res))
