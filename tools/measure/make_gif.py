"""README GIF: Original | Resolve Stabilizer | Trail Stability, cut from a 3240x1920 side-by-side render
(three 1080x1920 panels).  usage: make_gif.py render.mp4 first_frame n_frames out.gif [step=3] [panel_width=260]
Then reduce the palette:  ffmpeg -i out.gif -vf "split[a][b];[a]palettegen=max_colors=96[p];[b][p]paletteuse=dither=none" small.gif"""
import numpy as np, subprocess, sys
from PIL import Image, ImageDraw, ImageFont
src, f0, nfr, out = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
step = int(sys.argv[5]) if len(sys.argv) > 5 else 3          # 60 fps -> 20 fps
PW = int(sys.argv[6]) if len(sys.argv) > 6 else 260   # panel width; height keeps 9:16
PH = int(round(PW * 16 / 9 / 2)) * 2; GAP = 6; TOP = 34
labels = ["Original (Action Mode)", "Resolve Stabilizer", "Trail Stability"]
font = ImageFont.truetype("/usr/share/fonts/opentype/inter/Inter-SemiBold.otf", 15)
W = 3 * PW + 2 * GAP
p = subprocess.Popen(['ffmpeg', '-v', 'error', '-ss', f'{f0/60:.4f}', '-i', src, '-frames:v', str(nfr),
                      '-vf', f'scale={3*PW}:{PH}:flags=lanczos', '-pix_fmt', 'rgb24', '-f', 'rawvideo', '-'], stdout=subprocess.PIPE)
frames = []
k = 0
while True:
    b = p.stdout.read(3 * PW * PH * 3)
    if len(b) < 3 * PW * PH * 3: break
    if k % step == 0:
        a = np.frombuffer(b, np.uint8).reshape(PH, 3 * PW, 3)
        im = Image.new('RGB', (W, PH + TOP), (252, 252, 251))
        d = ImageDraw.Draw(im)
        for i in range(3):
            x = i * (PW + GAP)
            im.paste(Image.fromarray(a[:, i * PW:(i + 1) * PW]), (x, TOP))
            tw = d.textlength(labels[i], font=font)
            d.text((x + (PW - tw) / 2, 8), labels[i], fill=(11, 11, 11), font=font)
            # fixed reference lines: shake shows as movement against them
            for gy in (PH // 3, 2 * PH // 3):
                d.line([(x, TOP + gy), (x + PW - 1, TOP + gy)], fill=(255, 255, 255), width=1)
            d.line([(x + PW // 2, TOP), (x + PW // 2, TOP + PH - 1)], fill=(255, 255, 255), width=1)
        frames.append(im)
    k += 1
frames[0].save(out, save_all=True, append_images=frames[1:], duration=int(1000 * step / 60), loop=0, optimize=True)
print(out, len(frames))
