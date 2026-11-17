"""Download aerial photos (orthophotos) for sim_ground.py.

Run:  python fetch_ground.py ground      (skips sites already downloaded)

US sites: USGS The National Map, public domain, about 0.6 m of real detail per pixel.
Ottawa sites: City of Ottawa Basemap_Imagery services (2022 leaf-on 20 cm; 2019 fall). The service
states no licence, so use these photos only inside our simulator; do not publish them.
"""
import json
import math
import os
import sys
import urllib.request

import cv2
import numpy as np

USGS = "https://basemap.nationalmap.gov/arcgis/rest/services/USGSImageryOnly/MapServer/export"
OTTAWA = "https://maps.ottawa.ca/arcgis/rest/services/Basemap_Imagery_{}/MapServer/export"
SITES = {  # name: (lat, lon, export URL, ground size m, pixels)
    "prairie": (39.0925, -96.5750, USGS, 2400.0, 4096),      # Konza Prairie, KS: plain grass
    "farmland": (41.9500, -93.5500, USGS, 2400.0, 4096),     # central Iowa: fields, edges, roads
    "suburb": (38.9000, -94.6800, USGS, 2400.0, 4096),       # Overland Park, KS: houses, streets
    # HOLD-OUT sites (see DATA_SPLITS.md): final results only, never used to choose a setting.
    "holdout_quarry": (41.7960, -87.8360, USGS, 2400.0, 4096),   # large open quarry: rock, gravel, haul roads
    "holdout_field": (37.7683, -122.4927, USGS, 2400.0, 4096),   # park with playing fields and trees
    "holdout_water": (42.3555, -71.0760, USGS, 2400.0, 4096),    # built-up river bank: water beside streets
    "holdout_lawn": (40.7717, -73.9750, USGS, 2400.0, 4096),     # large city-park lawn: low texture under the hold point
    # Canadian War Museum, LeBreton Flats, Ottawa: the event site. 0.29 m per pixel.
    "ottawa2022": (45.4170, -75.7150, OTTAWA.format("2022"), 2400.0, 8192),
    "ottawa2019fall": (45.4170, -75.7150, OTTAWA.format("2019_Fall"), 2400.0, 8192),
}
MAX_PX = 4096        # the services' largest export
R = 6378137.0


def fetch(lat, lon, url, ground_m, px):
    """Export px x px pixels in MAX_PX tiles and stitch them (BGR array)."""
    k = 1 / math.cos(math.radians(lat))                # web-mercator metres per ground metre
    x0 = R * math.radians(lon) - ground_m / 2 * k
    y1 = R * math.log(math.tan(math.pi / 4 + math.radians(lat) / 2)) + ground_m / 2 * k
    n, tm = px // MAX_PX, ground_m / (px // MAX_PX) * k
    rows = []
    for i in range(n):
        row = []
        for j in range(n):
            bbox = f"{x0 + j * tm},{y1 - (i + 1) * tm},{x0 + (j + 1) * tm},{y1 - i * tm}"
            q = f"{url}?bbox={bbox}&bboxSR=3857&imageSR=3857&size={MAX_PX},{MAX_PX}&format=jpg&f=image"
            data = np.frombuffer(urllib.request.urlopen(q, timeout=120).read(), np.uint8)
            row.append(cv2.imdecode(data, cv2.IMREAD_COLOR))
        rows.append(np.hstack(row))
    return np.vstack(rows)


def main(out):
    path = os.path.join(out, "sites.json")
    meta = json.load(open(path)) if os.path.exists(path) else {}
    for name, (lat, lon, url, ground_m, px) in SITES.items():
        img_path = os.path.join(out, f"{name}.jpg")
        if not os.path.exists(img_path):
            cv2.imwrite(img_path, fetch(lat, lon, url, ground_m, px), [cv2.IMWRITE_JPEG_QUALITY, 92])
            print(name, os.path.getsize(img_path), "bytes", flush=True)
        meta[name] = dict(lat=lat, lon=lon, ground_m=ground_m, px=px, m_per_px=ground_m / px,
                          source=url.split("/services/")[0] + " " + url.split("/services/")[1].split("/")[0])
    json.dump(meta, open(path, "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1])
