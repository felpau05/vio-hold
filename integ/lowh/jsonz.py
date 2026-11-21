"""Read a result file of integ/lowh: plain .json or gzip-compressed .json.gz (the stored runs are compressed)."""
import gzip
import json


def jload(path):
    with (gzip.open(path, "rt") if path.endswith(".gz") else open(path)) as f:
        return json.load(f)


def jfind(stem):
    """The stored file for stem (a path without .json): stem.json or stem.json.gz."""
    import os
    for p in (stem + ".json", stem + ".json.gz"):
        if os.path.exists(p):
            return p
    raise FileNotFoundError(stem + ".json[.gz]")
