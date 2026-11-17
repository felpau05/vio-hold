import os, sys, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import common as C
def job(site, h0, seed, T):
    rec, _ = C.scenario(site, h0, T=T, seed=seed)
    d = C.run_stats(rec, h0); d.pop("e2")
    s = rec.sent
    d.update(site=site, seed=seed, n_med=float(np.median(rec.n)), whys=sorted(set(rec.why[rec.why != ""]))[:4],
             states={k: int(np.sum(rec.state == k)) for k in ("ok", "degraded", "lost", "init")},
             err_t=[(round(float(t),1), round(float(e),2)) for t, e in zip(rec.t[::60], rec.err[::60])])
    return d
if __name__ == "__main__":
    jobs = [dict(site="prairie", h0=50, seed=k, T=20.0) for k in range(1, 7)]
    for r in C.pmap(job, jobs):
        print(r["seed"], C.fmt(r), "n_med", r["n_med"], r["states"], r["whys"])
        print("   ", r["err_t"])
