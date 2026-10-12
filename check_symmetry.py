import re
import numpy as np
import analyze_transform as A

txt = A.TXT.read_text(encoding="utf-8", errors="replace")
geoms = A.parse("GEOM", txt)
bodies = A.parse("BODY", txt)
mbi = {int(i): n for i, n in re.findall(r"MESH (\d+):\n  name\s+(\S+)", txt)}


def bw(bid):
    ch = []
    b = bid
    while b > 0:
        ch.append(b)
        b = int(bodies[b]["body_parentid"][0])
    ch.reverse()
    R = np.eye(3)
    t = np.zeros(3)
    for bb in ch:
        bp = np.array([float(x) for x in bodies[bb]["body_pos"]])
        bq = [float(x) for x in bodies[bb]["body_quat"]]
        t = t + R @ bp
        R = R @ A.q2m(bq)
    return t, R


res = {}
for gid, g in geoms.items():
    if g.get("geom_group", [""])[0] != "2":
        continue
    if g.get("geom_type", [""])[0] != "7":
        continue
    nm = mbi[int(g["geom_dataid"][0])]
    V = A.read_stl(A.MESH_DIR / (nm + ".STL"))
    bt, bR = bw(int(g["geom_bodyid"][0]))
    Rw = bR @ A.q2m([float(x) for x in g["geom_quat"]])
    tw = bt + bR @ np.array([float(x) for x in g["geom_pos"]])
    res[nm] = (Rw @ V.T).T + tw

print("total meshes:", len(res))
print("\n=== mirror check (left vs mirrored right) ===")
for base in ["hip_y", "hip_x", "hip_z", "knee_y", "ankle_y", "ankle_x",
             "shoulder_y", "shoulder_x", "shoulder_z", "elbow_y",
             "wrist_x", "wrist_y", "wrist_z"]:
    l = res.get("l_" + base)
    r = res.get("r_" + base)
    if l is None or r is None:
        print("MISSING", base, "L:", l is None, "R:", r is None)
        continue
    lc = (l.min(0) + l.max(0)) / 2
    rc = (r.min(0) + r.max(0)) / 2
    rcm = np.array([rc[0], -rc[1], rc[2]])
    d = float(np.linalg.norm(lc - rcm))
    print(("OK  " if d < 0.02 else "BAD "), f"{base:12s} L=({lc[0]:+.3f},{lc[1]:+.3f},{lc[2]:+.3f}) Rm=({rcm[0]:+.3f},{rcm[1]:+.3f},{rcm[2]:+.3f}) d={d:.4f}")

print("\n=== full body height map (MuJoCo world Z, robot stands upright) ===")
for nm in sorted(res):
    b = res[nm]
    print(f"{nm:20s} X[{b[:,0].min():+.3f},{b[:,0].max():+.3f}] "
          f"Y[{b[:,1].min():+.3f},{b[:,1].max():+.3f}] "
          f"Z[{b[:,2].min():+.3f},{b[:,2].max():+.3f}]")
