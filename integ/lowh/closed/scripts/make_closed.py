#!/usr/bin/env python3
"""Before/after table from ab_before and ab_after apsim/low_ap_results.json (closed-loop A/B of proposed.patch)."""
import json
import os
import re
import statistics as st

SP = os.path.dirname(os.path.abspath(__file__))
V = [("before", "ab_before"), ("after", "ab_after")]
KEYS = ("ailsafe", "EKF", "variance", "Dead Reckoning", "lane", "aiding", "position", "stopped", "Vision", "visual")


def load(copy):
    try:
        return json.load(open(os.path.join(SP, copy, "apsim", "low_ap_results.json")))
    except (OSError, ValueError):
        return {}


def drv_load():
    """1-min load at each run's start, from the driver log (the last 'start' per copy and label)."""
    out = {}
    for line in open(os.path.join(SP, "ab_driver.log")):
        e = json.loads(line)
        if e["ev"] == "start":
            out[(e["copy"], e["label"])] = e["load"][0]
    return out


def row(r):
    T = r.get("hold_s_flown") or 0
    w = r.get("why_not_sent_hold") or {}
    re_n = sum(1 for t, x in (r.get("notes") or []) if "re-anchored" in x and 0 <= t <= T)
    modes = [m for t, m in (r.get("fc_modes") or []) if t > 0.5]
    left = [e for e in (r.get("events") or []) if "left Loiter" in str(e[1]) or "FLY-AWAY" in str(e[1])
            or "crashed" in str(e[1]) or "ERROR" in str(e[1]) or "clock stopped" in str(e[1])]
    loiter = "LOITER" if not modes and not left and T >= 59.5 else "NO: " + "; ".join(
        [f"{t}s {m}" for t, m in (r.get("fc_modes") or []) if t > 0.5] + [f"{t}s {x}" for t, x in left])
    txt = [(t, x) for t, x in (r.get("texts_all") or []) if 0 <= t <= T + 1 and any(k in x for k in KEYS)]
    return dict(rms=r.get("rms"), max=r.get("max"), hz=r.get("fix_rate_hz"), gap=r.get("fix_gap_max_s"),
                halves=w.get("whole picture: halves disagree", 0), points=w.get("whole picture and points disagree", 0),
                reanch=re_n, loiter=loiter, ekf=txt, load=(r.get("load_start") or [None])[0], T=T,
                gaps1=r.get("fix_gaps_over_1s"), ferr=r.get("ferr_rms_60s"), pct=r.get("pct_frames_fix"))


def fmt(x, p=2, n=6):
    return f"{x:{n}.{p}f}" if isinstance(x, (int, float)) else "-".rjust(n)


def main():
    res = {v: load(c) for v, c in V}
    dl = drv_load()
    conds = [("5 m, wind 20 gust 40", r"lowap_apb_5m_w20g40_s(\d+)$"), ("10 m, wind 20 gust 40", r"lowap_apb_10m_w20g40_s(\d+)$"),
             ("5 m, calm", r"lowap_apb_5m_calm_s(\d+)$")]
    hdr = ("condition / seed".ljust(34) + "ver".ljust(7) + "rms m".rjust(6) + "max m".rjust(6) + "fix Hz".rjust(7) +
           "gap s".rjust(6) + "halves".rjust(7) + "points".rjust(7) + "re-anc".rjust(7) + "load".rjust(6) +
           "  FC mode; EKF/failsafe texts in hold")
    out = [hdr, "-" * len(hdr)]
    for name, pat in conds:
        seeds = sorted({int(re.match(pat, k).group(1)) for v in res for k in res[v] if re.match(pat, k)})
        agg = {v: [] for v, _ in V}
        for s in seeds:
            for v, c in V:
                lab = re.sub(r"\(\\d\+\)\$$", str(s), pat)
                r = res[v].get(lab)
                if r is None:
                    out.append(f"{name} s{s}".ljust(34) + v.ljust(7) + "  (no finished run)")
                    continue
                x = row(r)
                if x["T"] < 59.5:
                    out.append(f"{name} s{s}".ljust(34) + v.ljust(7) + f"  (did not finish: {r.get('events')})")
                    continue
                agg[v].append(x)
                ld = dl.get((c, lab), x["load"])
                out.append(f"{name} s{s}".ljust(34) + v.ljust(7) + fmt(x["rms"]) + fmt(x["max"]) + fmt(x["hz"], 1, 7) +
                           fmt(x["gap"]) + f"{x['halves']:7d}" + f"{x['points']:7d}" + f"{x['reanch']:7d}" +
                           fmt(ld, 1) + "  " + x["loiter"] + ("; texts: " + "; ".join(f"{t}s {y}" for t, y in x["ekf"])
                                                             if x["ekf"] else "; none"))
        for v, _ in V:
            a = agg[v]
            if len(a) >= 2:
                m = lambda k: st.median([x[k] for x in a])            # noqa: E731
                rg = lambda k: f"{min(x[k] for x in a):.4g}-{max(x[k] for x in a):.4g}"   # noqa: E731
                out.append(f"{name} median (n={len(a)})".ljust(34) + v.ljust(7) + fmt(m("rms")) + fmt(m("max")) +
                           fmt(m("hz"), 1, 7) + fmt(m("gap")) + fmt(m("halves"), 0, 7) + fmt(m("points"), 0, 7) +
                           fmt(m("reanch"), 0, 7) + "".rjust(6) +
                           f"  range rms {rg('rms')}; max {rg('max')}; gap {rg('gap')}; halves {rg('halves')}")
        out.append("")
    # per-run extra facts
    out.append("Extra per run: % sent fixes per rendered frame; sent-fix error RMS over the hold (ferr60, m); 1-min load at the")
    out.append("end; other SITL / px4_sim at the end; gaps over 1 s [start s after the switch, length s]")
    for v, c in V:
        for k in sorted(res[v]):
            r = res[v][k]
            x = row(r)
            le = (r.get("load_end") or [None])[0]
            out.append(f"  {v:6s} {k:26s} %fix {fmt(x['pct'], 1)}  ferr60 {fmt(x['ferr'])}  load end {fmt(le, 1, 4)}  "
                       f"jobs end {r.get('jobs_end')}  gaps>1s {x['gaps1']}")
    print("\n".join(out))


if __name__ == "__main__":
    main()
