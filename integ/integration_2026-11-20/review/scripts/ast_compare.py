"""Compare each top-level statement of two Python files by syntax tree (ast.dump, no line numbers)."""
import ast, sys
def defs(p):
    t = ast.parse(open(p).read())
    out = {}
    for i, n in enumerate(t.body):
        name = getattr(n, "name", None) or (n.targets[0].id if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name) else f"<stmt {type(n).__name__} {i}>")
        if isinstance(n, (ast.Import, ast.ImportFrom)):
            name = "import " + ",".join(a.name for a in n.names)
        out[name] = ast.dump(n, include_attributes=False)
    return out
a, b = defs(sys.argv[1]), defs(sys.argv[2])
same = [k for k in a if k in b and a[k] == b[k]]
changed = [k for k in a if k in b and a[k] != b[k]]
print("only old:", [k for k in a if k not in b])
print("only new:", [k for k in b if k not in a])
print("changed:", changed)
print(f"same: {len(same)}")
loop = ["mode_run", "run_live", "Pipe", "Engage", "Raw", "Log", "catch_up", "GyroAtt", "ImuAtt", "att_filter", "live_gate",
        "start_problems", "make_hold", "lens", "load_mask", "Speed", "WindAssist", "OutageTest", "RateGate", "stamp",
        "delay_advice", "fc_mode", "_join", "open_video", "open_fc", "NO_FRAME_S", "AP_MODES", "AP_HOLD", "PX4_MAIN"]
for k in loop:
    print(f"  {k}: {'same' if k in same else 'CHANGED' if k in changed else 'missing in ' + ('old' if k not in a else 'new')}")
