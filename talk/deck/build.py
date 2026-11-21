"""Build talk/deck/index.html (a standalone, offline-capable deck) from talk/deck/project/.

  ~/.venvs/vio/bin/python talk/deck/build.py
"""
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.join(HERE, "project")
# uploaded copies in the claude.ai deck -> the figure files in this repo
BLOB = {"d14b1a9ed7a676b3e8fcc3e77ca61199": "01_gyro_camera_offset.png", "dc2d7759bbdfa8471565d5dac7e63cfb": "02_frame_stamps.png",
        "b4d0f8e79fca840ddb11dfa6160921d8": "03_lens_measured.png", "f874c0d21da79a95fe7659d6a5d2f353": "04_first_walk.png",
        "af39b11d60e81ef26148b5923e00ebd3": "05_compute.png"}

SHELL = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>__TITLE__</title>__FONTS__
<style>
* { margin: 0; }
html, body { height: 100%; background: #0d1319; overflow: hidden; }
#stage { position: absolute; left: 50%; top: 50%; width: 1920px; height: 1080px; transform-origin: 0 0; }
#stage > section { position: absolute; inset: 0; width: 1920px; height: 1080px; display: none; }
#stage > section.on { display: flex; }
aside { display: none; }
body.notes aside { display: block; position: absolute; left: 0; right: 0; bottom: 0; padding: 20px 128px; font-size: 26px;
  line-height: 1.4; background: rgba(13,19,25,.92); color: #EEF1F4; font-family: Inter, Arial, sans-serif; }
.arrow { clip-path: polygon(0 30%, 60% 30%, 60% 0, 100% 50%, 60% 100%, 60% 70%, 0 70%); flex: none; }
#pg { position: fixed; right: 16px; bottom: 12px; color: #A9B4C0; font: 14px Inter, Arial, sans-serif; }
__CSS__
</style></head><body>
<div id="stage">
__SLIDES__
</div>
<div id="pg"></div>
<script>
const S = [...document.querySelectorAll('#stage > section')], stage = document.getElementById('stage');
let i = Math.max(0, Math.min(S.length - 1, (parseInt(location.hash.slice(1)) || 1) - 1));
function fit() { const k = Math.min(innerWidth / 1920, innerHeight / 1080);
  stage.style.transform = `scale(${k}) translate(-50%, -50%)`; }
function show(n) { i = Math.max(0, Math.min(S.length - 1, n)); S.forEach((s, j) => s.classList.toggle('on', j === i));
  document.getElementById('pg').textContent = `${i + 1} / ${S.length}  ·  arrows to move, F full screen, N notes, B backup`;
  history.replaceState(null, '', '#' + (i + 1)); }
addEventListener('keydown', e => {
  if (['ArrowRight', 'PageDown', ' '].includes(e.key)) show(i + 1);
  else if (['ArrowLeft', 'PageUp'].includes(e.key)) show(i - 1);
  else if (e.key === 'Home') show(0);
  else if (e.key === 'End') show(S.length - 1);
  else if (e.key === 'b' || e.key === 'B') show(S.findIndex(s => s.id === 'backup'));
  else if (e.key === 'f' || e.key === 'F') document.fullscreenElement ? document.exitFullscreen() : document.documentElement.requestFullscreen();
  else if (e.key === 'n' || e.key === 'N') document.body.classList.toggle('notes'); });
addEventListener('click', e => show(e.clientX > innerWidth / 3 ? i + 1 : i - 1));
addEventListener('resize', fit); fit(); show(i);
</script></body></html>
"""


def main():
    deck = json.load(open(os.path.join(D, "deck.json")))
    slides = []
    for sid in deck["order"]:
        h = open(os.path.join(D, "slides", f"{sid}.html")).read()
        h = re.sub(r'src="/_blob/([0-9a-f]{32})"', lambda m: f'src="../stage2/{BLOB[m.group(1)]}"', h)
        h = re.sub(r'<x-shape kind="arrow-right" style="([^"]*)"></x-shape>',
                   lambda m: f'<div class="arrow" style="{m.group(1)}"></div>', h)
        slides.append(h)
    fonts = "".join(f'<link rel="stylesheet" href="{f["href"]}">' for f in deck["faces"].values())
    page = (SHELL.replace("__TITLE__", deck["title"]).replace("__FONTS__", fonts)
            .replace("__CSS__", open(os.path.join(D, "deck.css")).read()).replace("__SLIDES__", "\n".join(slides)))
    out = os.path.join(HERE, "index.html")
    open(out, "w").write(page)
    print(f"wrote {out}: {len(slides)} slides")


if __name__ == "__main__":
    main()
