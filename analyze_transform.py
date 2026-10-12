"""分析 MuJoCo geom 位姿 -> three.js 渲染的坐标变换是否正确。

读入 compile.exe 导出的 ASCII 模型（compiled_ascii.txt），
对每个 visual mesh geom：
  1. 计算 geom 世界位姿（body 链 + geom 相对位姿）
  2. 把 STL 顶点用 MuJoCo 语义变换到世界系（Z-up）
  3. 再按演示脚本的 (x,z,-y) + R'=M R M^T 变换到 three.js 语义（Y-up）
  4. 检查各部件世界包围盒是否连续（脚在低处、头在高处、左右对称）

以此判断 demo 脚本的变换公式是否会让零件错位/解体。
"""
import re
import struct
import os
import numpy as np

import pathlib
ROOT = pathlib.Path(__file__).parent
MESH_DIR = ROOT / "meshes"
TXT = ROOT / "compiled_ascii.txt"


def parse(section, txt):
    out = {}
    parts = re.split(r"\n" + section + r" (\d+):\n", txt)
    for i in range(1, len(parts), 2):
        idx = int(parts[i])
        blk = parts[i + 1]
        d = {}
        for m in re.finditer(r"\n  (\w+)\s+([-\d\.eE ]+)", blk):
            d[m.group(1)] = m.group(2).split()
        out[idx] = d
    return out


def q2m(q):
    """MuJoCo (w,x,y,z) -> rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def read_stl(p):
    with open(p, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        V = []
        for _ in range(n):
            d = f.read(50)
            if len(d) < 50:
                break
            v = struct.unpack("<12f", d[:48])
            V += [v[3:6], v[6:9], v[9:12]]
        return np.array(V)


def main():
    txt = TXT.read_text(encoding="utf-8", errors="replace")
    geoms = parse("GEOM", txt)
    bodies = parse("BODY", txt)
    meshes = {}
    for m in re.finditer(r"\nMESH (\d+):\n(?:  \w+\s+[^\n]*\n)*?  name\s+(\S+)", txt):
        pass
    # simpler: mesh index order == asset declaration order (names)
    mesh_names = re.findall(r"\nMESH (\d+):\n  name\s+(\S+)", txt)
    mesh_by_idx = {int(i): nm for i, nm in mesh_names}
    print(f"parsed: {len(geoms)} geoms, {len(bodies)} bodies, {len(mesh_by_idx)} meshes")

    def body_world(bid):
        chain = []
        b = bid
        while b > 0:
            chain.append(b)
            b = int(bodies[b]["body_parentid"][0])
        chain.reverse()
        R = np.eye(3)
        t = np.zeros(3)
        for bb in chain:
            bp = np.array([float(x) for x in bodies[bb]["body_pos"]])
            bq = [float(x) for x in bodies[bb]["body_quat"]]
            t = t + R @ bp
            R = R @ q2m(bq)
        return t, R

    M = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])

    rows = []
    for gid, g in sorted(geoms.items()):
        if g.get("geom_group", [""])[0] != "2":
            continue
        if g.get("geom_type", [""])[0] != "7":
            continue
        did = int(g["geom_dataid"][0])
        nm = mesh_by_idx.get(did, f"mesh{did}")
        stl = MESH_DIR / f"{nm}.STL"
        if not stl.exists():
            rows.append((gid, nm, None, None))
            continue
        V = read_stl(stl)
        bt, bR = body_world(int(g["geom_bodyid"][0]))
        gp = np.array([float(x) for x in g["geom_pos"]])
        gq = [float(x) for x in g["geom_quat"]]
        gR = q2m(gq)
        # geom world (MuJoCo Z-up)
        Rw = bR @ gR
        tw = bt + bR @ gp
        Vworld = (Rw @ V.T).T + tw
        # three.js semantics per demo script
        Vthree = Vworld @ M.T          # (x,y,z)->(x,z,-y)
        rows.append((gid, nm, Vworld, Vthree))

    print("\n%-4s %-20s %-28s %-28s %s" % ("gid", "mesh", "MuJoCo world Z-up", "three.js (x,z,-y) + RMRt", "delta_mid"))
    for gid, nm, Vw, Vt in rows:
        if Vw is None:
            print(f"{gid:<4} {nm:<20} MISSING STL")
            continue
        b1 = Vw.min(0), Vw.max(0)
        b2 = Vt.min(0), Vt.max(0)
        c1 = (b1[0] + b1[1]) / 2
        c2 = (b2[0] + b2[1]) / 2
        print(f"{gid:<4} {nm:<20} "
              f"Z[{b1[0][2]:+.3f},{b1[1][2]:+.3f}] "
              f"Ythree[{b2[0][1]:+.3f},{b2[1][1]:+.3f}] "
              f"Xthree[{b2[0][0]:+.3f},{b2[1][0]:+.3f}]")

    # global bbox
    allw = np.vstack([Vw for _, _, Vw, _ in rows if Vw is not None])
    allt = np.vstack([Vt for _, _, _, Vt in rows if Vt is not None])
    print("\nGlobal MuJoCo-world bbox  min", allw.min(0).round(3), "max", allw.max(0).round(3))
    print("Global three.js     bbox  min", allt.min(0).round(3), "max", allt.max(0).round(3))


if __name__ == "__main__":
    main()
