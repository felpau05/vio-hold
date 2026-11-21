The talk's slide deck.

  talk/deck/index.html     open in any browser (arrows to move, F full screen, N speaker notes). Uses talk/stage2/*.png.
  talk/deck/project/       the slide source (one HTML file per slide, shared deck.css, deck.json for the order).

To change a slide, edit project/slides/<id>.html (or deck.css), then rebuild index.html:
  ~/.venvs/vio/bin/python talk/deck/build.py
Figures: ~/.venvs/vio/bin/python talk/make_stage2_figures.py

Live preview while editing (rebuilds on save, the browser tab reloads by itself on the same slide):
  ~/.venvs/vio/bin/python talk/deck/serve.py      then open http://localhost:8000/deck/   (Ctrl+C to stop)
