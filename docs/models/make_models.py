#!/usr/bin/env python3
"""Generates libfpv's drone models in engine-neutral formats, for game
integrations to convert into each game's native model format (see README.md
next to this file and INTEGRATION.md "The drone model").

    python3 -I docs/models/make_models.py            # every model
    python3 -I docs/models/make_models.py quad5      # one model

For each model it writes docs/models/<name>/:
  <name>.gltf   glTF 2.0, buffer embedded (one file). Blender, Sollumz,
                CodeWalker/CodeX importers, Unity, Godot... read it.
  <name>.obj    Wavefront OBJ + <name>.mtl, the same geometry, for tools
                without glTF.
  <name>.json   what a converter needs to know: axes, units, parts (mesh
                names), each motor by libfpv's motor index, the camera,
                the materials' colours.

Standard library only. Geometry is built in metres in the model frame
"X forward, Y left, Z up" (the frame the first model was drawn in, for
Portal 2) and written out in glTF's own convention: +Y up, +Z forward
(glTF's "front"), so +X is the model's LEFT. Both frames are right-handed,
the change is a rotation (no mirror): winding and normals stay valid.
"""
import base64
import json
import math
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


# --- small vector helpers ------------------------------------------------------------------

def add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def scale(a, k): return (a[0] * k, a[1] * k, a[2] * k)
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    l = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2) or 1
    return scale(a, 1 / l)


def basis(axis):
    t = (1, 0, 0) if abs(axis[0]) < 0.9 else (0, 1, 0)
    u = norm(cross(axis, t))
    return u, cross(axis, u)


def to_gltf(p):
    """Model frame (X forward, Y left, Z up) -> glTF (X left, Y up, Z forward)."""
    return (p[1], p[2], p[0])


# --- meshes ---------------------------------------------------------------------------------

class Mesh:
    """Triangles with a material each, in the model frame, metres."""

    def __init__(self):
        self.tris = []  # (material, [(pos, normal, uv) x3])

    def tri(self, mat, a, b, c, n=None, uvs=((0, 0), (1, 0), (1, 1))):
        if n is None:
            n = norm(cross(sub(b, a), sub(c, a)))
        self.tris.append((mat, [(a, n, uvs[0]), (b, n, uvs[1]), (c, n, uvs[2])]))

    def quad(self, mat, a, b, c, d, n=None):
        """a b c d counter-clockwise seen from the side the normal faces."""
        self.tri(mat, a, b, c, n, ((0, 0), (1, 0), (1, 1)))
        self.tri(mat, a, c, d, n, ((0, 0), (1, 1), (0, 1)))

    def box(self, mat, center, half, axes=None, side_mats=None):
        """An oriented box: half extents along its axes (default X, Y, Z)."""
        ax = axes or ((1, 0, 0), (0, 1, 0), (0, 0, 1))

        def corner(sx, sy, sz):
            return add(center, add(scale(ax[0], sx * half[0]), add(scale(ax[1], sy * half[1]), scale(ax[2], sz * half[2]))))

        faces = [  # (normal axis, sign), corners counter-clockwise from outside
            ((0, 1), [(1, -1, -1), (1, 1, -1), (1, 1, 1), (1, -1, 1)]),
            ((0, -1), [(-1, 1, -1), (-1, -1, -1), (-1, -1, 1), (-1, 1, 1)]),
            ((1, 1), [(1, 1, -1), (-1, 1, -1), (-1, 1, 1), (1, 1, 1)]),
            ((1, -1), [(-1, -1, -1), (1, -1, -1), (1, -1, 1), (-1, -1, 1)]),
            ((2, 1), [(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]),
            ((2, -1), [(-1, 1, -1), (1, 1, -1), (1, -1, -1), (-1, -1, -1)]),
        ]
        for i, ((axis, sign), cs) in enumerate(faces):
            n = scale(ax[axis], sign)
            m = side_mats.get(i, mat) if side_mats else mat
            self.quad(m, *[corner(*k) for k in cs], n=n)

    def cylinder(self, mat, base, radius, height, segments=16, cap_mat=None, axis=(0, 0, 1)):
        u, v = basis(axis)
        top = add(base, scale(axis, height))
        for i in range(segments):
            a0, a1 = 2 * math.pi * i / segments, 2 * math.pi * (i + 1) / segments
            d0 = add(scale(u, math.cos(a0)), scale(v, math.sin(a0)))
            d1 = add(scale(u, math.cos(a1)), scale(v, math.sin(a1)))
            p0, p1 = add(base, scale(d0, radius)), add(base, scale(d1, radius))
            q0, q1 = add(top, scale(d0, radius)), add(top, scale(d1, radius))
            self.quad(mat, p0, p1, q1, q0, n=norm(add(d0, d1)))
            cm = cap_mat or mat
            self.tri(cm, top, q0, q1, n=axis)
            self.tri(cm, base, p1, p0, n=scale(axis, -1))

    def disc(self, mat, center, radius, segments=28):
        for i in range(segments):
            a0, a1 = 2 * math.pi * i / segments, 2 * math.pi * (i + 1) / segments
            p0 = add(center, (radius * math.cos(a0), radius * math.sin(a0), 0))
            p1 = add(center, (radius * math.cos(a1), radius * math.sin(a1), 0))
            uv = lambda a: (0.5 + 0.5 * math.cos(a), 0.5 + 0.5 * math.sin(a))
            self.tri(mat, center, p0, p1, n=(0, 0, 1), uvs=((0.5, 0.5), uv(a0), uv(a1)))


# --- quad5: a 5" freestyle quad -------------------------------------------------------------
# First drawn for portal-2-fpv (its model/build_model.py, in inches); here in
# metres. Sizes follow libfpv's freestyler-5 preset: motors 0.0795 m from the
# centre on both axes, 5.1" props.

IN = 0.0254  # the original was laid out in inches (Source units)


def quad5():
    motor = 0.0795
    prop_r = 0.0635 + 0.0013  # 5.1" props
    prop_z = 0.62 * IN
    # libfpv's motor index i is Betaflight's motor i + 1 (Quad X):
    # 0 rear right, 1 front right, 2 rear left, 3 front left. (x forward, y left)
    motors = [(-motor, -motor), (motor, -motor), (-motor, motor), (motor, motor)]
    spin = [+1, -1, -1, +1]  # Betaflight's default (props in), seen from above: +1 clockwise

    body = Mesh()
    for (x, y) in motors:  # the X frame's four arms
        d = norm((x, y, 0))
        length = math.hypot(x, y)
        body.box("carbon", (x / 2, y / 2, -0.55 * IN), (length / 2 + 0.3 * IN, 0.32 * IN, 0.1 * IN),
                 axes=(d, (-d[1], d[0], 0), (0, 0, 1)))
    body.box("electronics", (0, 0, -0.12 * IN), (0.95 * IN, 0.75 * IN, 0.35 * IN))  # the stack
    body.box("carbon", (0, 0, 0.32 * IN), (1.5 * IN, 0.85 * IN, 0.07 * IN))  # top plate
    for sx in (-1, 1):
        for sy in (-1, 1):
            body.cylinder("metal", (sx * 1.2 * IN, sy * 0.65 * IN, -0.45 * IN), 0.08 * IN, 0.72 * IN, 6)
    # the battery: white with a blue and an orange stripe, a black strap
    body.box("white", (-0.15 * IN, 0, 0.98 * IN), (1.45 * IN, 0.72 * IN, 0.58 * IN), side_mats={2: "blue", 3: "orange"})
    body.box("strap", (-0.15 * IN, 0, 0.98 * IN), (0.22 * IN, 0.76 * IN, 0.61 * IN))
    # the camera, tilted up 25 degrees (the preset's uptilt), and its lens
    t = math.radians(25)
    fwd, up = (math.cos(t), 0, math.sin(t)), (-math.sin(t), 0, math.cos(t))
    cam = (1.45 * IN, 0, -0.05 * IN)
    body.box("camera", cam, (0.38 * IN, 0.4 * IN, 0.38 * IN), axes=(fwd, (0, 1, 0), up))
    lens = add(cam, scale(fwd, 0.38 * IN))
    body.cylinder("lens", lens, 0.26 * IN, 0.22 * IN, 14, axis=fwd)
    a = norm((-0.5, 0, 1))  # antenna
    body.cylinder("white", (-1.45 * IN, 0, 0.35 * IN), 0.06 * IN, 1.3 * IN, 6, axis=a)
    for (x, y) in motors:  # motors: base, bell, shaft
        body.cylinder("metal", (x, y, -0.66 * IN), 0.5 * IN, 0.2 * IN, 16)
        body.cylinder("motor", (x, y, -0.46 * IN), 0.52 * IN, 0.72 * IN, 18, cap_mat="metal")
        body.cylinder("metal", (x, y, 0.26 * IN), 0.1 * IN, 0.42 * IN, 6)

    parts = {"body": body}
    for i, (x, y) in enumerate(motors):
        still = Mesh()  # two blades and the hub
        ang = math.radians(30 + 60 * i)
        d = (math.cos(ang), math.sin(ang), 0)
        still.box("prop", (x, y, prop_z), (prop_r, 0.24 * IN, 0.035 * IN), axes=(d, (-d[1], d[0], 0), (0, 0, 1)))
        still.cylinder("prop", (x, y, prop_z - 0.06 * IN), 0.28 * IN, 0.16 * IN, 10)
        spinning = Mesh()  # the blur disc and the hub
        spinning.disc("propblur", (x, y, prop_z), prop_r)
        spinning.cylinder("prop", (x, y, prop_z - 0.06 * IN), 0.28 * IN, 0.16 * IN, 10)
        parts["prop_still_%d" % i] = still
        parts["prop_spin_%d" % i] = spinning

    info = {
        "description": "A 5\" freestyle FPV quad (libfpv's freestyler-5 preset): true-X frame, 2207 motors, "
                       "5.1\" props, a 6S battery on top, a camera tilted up 25 degrees.",
        "origin": "the drone's centre: libfpv's frame.position (the motors' hubs are about level with it)",
        "motors": [{"index": i, "where": ["rear right", "front right", "rear left", "front left"][i],
                    "hub": {"forward": round(x, 4), "left": round(y, 4), "up": 0.0},
                    "spin": "clockwise" if spin[i] > 0 else "counter-clockwise",
                    "parts": ["prop_still_%d" % i, "prop_spin_%d" % i]} for i, (x, y) in enumerate(motors)],
        "camera": {"lens": {"forward": round(lens[0], 4), "left": 0.0, "up": round(lens[2], 4)}, "uptilt_deg": 25},
        "size_m": {"forward": 0.27, "left": 0.27, "up": 0.06},
    }
    return parts, info


MODELS = {"quad5": quad5}

COLORS = {  # material: (r, g, b, a), sRGB 0..255
    "carbon": (34, 34, 38, 255),
    "electronics": (24, 60, 32, 255),
    "metal": (150, 152, 158, 255),
    "motor": (70, 72, 80, 255),
    "white": (236, 236, 232, 255),
    "blue": (40, 140, 255, 255),
    "orange": (255, 140, 30, 255),
    "strap": (20, 20, 22, 255),
    "camera": (18, 18, 20, 255),
    "lens": (30, 40, 60, 255),
    "prop": (40, 40, 44, 255),
    "propblur": (60, 60, 66, 70),
}


# --- writers --------------------------------------------------------------------------------

def srgb_to_linear(c):
    c /= 255.0
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def write_gltf(path, name, parts):
    mats = sorted({m for mesh in parts.values() for m, _ in mesh.tris})
    mat_index = {m: i for i, m in enumerate(mats)}
    buf = bytearray()
    views, accessors, meshes, nodes = [], [], [], []

    def add_view(data, target):
        while len(buf) % 4:
            buf.append(0)
        views.append({"buffer": 0, "byteOffset": len(buf), "byteLength": len(data), "target": target})
        buf.extend(data)
        return len(views) - 1

    for part, mesh in parts.items():
        prims = []
        for m in mats:
            verts = [v for mm, vs in mesh.tris if mm == m for v in vs]
            if not verts:
                continue
            pos = [to_gltf(p) for p, _, _ in verts]
            nrm = [to_gltf(n) for _, n, _ in verts]
            uvs = [(u, 1 - v) for _, _, (u, v) in verts]
            lo = [min(p[k] for p in pos) for k in range(3)]
            hi = [max(p[k] for p in pos) for k in range(3)]
            pv = add_view(b"".join(struct.pack("<3f", *p) for p in pos), 34962)
            nv = add_view(b"".join(struct.pack("<3f", *n) for n in nrm), 34962)
            tv = add_view(b"".join(struct.pack("<2f", *t) for t in uvs), 34962)
            base = len(accessors)
            accessors += [
                {"bufferView": pv, "componentType": 5126, "count": len(pos), "type": "VEC3", "min": lo, "max": hi},
                {"bufferView": nv, "componentType": 5126, "count": len(nrm), "type": "VEC3"},
                {"bufferView": tv, "componentType": 5126, "count": len(uvs), "type": "VEC2"},
            ]
            prims.append({"attributes": {"POSITION": base, "NORMAL": base + 1, "TEXCOORD_0": base + 2},
                          "material": mat_index[m]})
        meshes.append({"name": part, "primitives": prims})
        nodes.append({"name": part, "mesh": len(meshes) - 1})

    materials = []
    for m in mats:
        r, g, b, a = COLORS[m]
        mat = {"name": m, "pbrMetallicRoughness": {
            "baseColorFactor": [srgb_to_linear(r), srgb_to_linear(g), srgb_to_linear(b), a / 255.0],
            "metallicFactor": 0.6 if m in ("metal", "motor") else 0.0, "roughnessFactor": 0.5}}
        if a < 255:
            mat["alphaMode"] = "BLEND"
            mat["doubleSided"] = True
        materials.append(mat)

    gltf = {
        "asset": {"version": "2.0", "generator": "libfpv docs/models/make_models.py"},
        "scene": 0,
        "scenes": [{"name": name, "nodes": [len(nodes)]}],
        "nodes": nodes + [{"name": name, "children": list(range(len(nodes)))}],
        "meshes": meshes,
        "materials": materials,
        "accessors": accessors,
        "bufferViews": views,
        "buffers": [{"byteLength": len(buf),
                     "uri": "data:application/octet-stream;base64," + base64.b64encode(bytes(buf)).decode()}],
    }
    with open(path, "w") as f:
        json.dump(gltf, f, separators=(",", ":"))


def write_obj(path, name, parts):
    mtl = os.path.splitext(os.path.basename(path))[0] + ".mtl"
    lines = ["# %s: libfpv docs/models/make_models.py" % name,
             "# metres; +Y up, +Z forward (the nose), +X left (as the .gltf)", "mtllib " + mtl]
    vi = 1
    for part, mesh in parts.items():
        lines.append("o " + part)
        cur = None
        for m, verts in mesh.tris:
            if m != cur:
                lines.append("usemtl " + m)
                cur = m
            for p, n, (u, v) in verts:
                p, n = to_gltf(p), to_gltf(n)
                lines.append("v %.5f %.5f %.5f" % p)
                lines.append("vn %.4f %.4f %.4f" % n)
                lines.append("vt %.4f %.4f" % (u, v))
            lines.append("f %d/%d/%d %d/%d/%d %d/%d/%d" % (vi, vi, vi, vi + 1, vi + 1, vi + 1, vi + 2, vi + 2, vi + 2))
            vi += 3
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    with open(os.path.join(os.path.dirname(path), mtl), "w") as f:
        for m, (r, g, b, a) in COLORS.items():
            f.write("newmtl %s\nKd %.4f %.4f %.4f\nd %.4f\n\n" % (m, r / 255, g / 255, b / 255, a / 255))


def check(name, parts, info):
    """The axes must come out as documented: catches a model turned or mirrored."""
    def centroid(mat):
        pts = [to_gltf(p) for mesh in parts.values() for m, vs in mesh.tris if m == mat for p, _, _ in vs]
        return [sum(p[k] for p in pts) / len(pts) for k in range(3)]
    lens = centroid("lens")
    assert lens[2] > 0.03 and abs(lens[0]) < 1e-6, "%s: the lens must be at +Z (forward)" % name
    battery = centroid("strap")
    assert battery[1] > 0.01, "%s: the battery must be at +Y (up)" % name
    for mo in info["motors"]:
        prop = centroid_of(parts["prop_still_%d" % mo["index"]])
        want = to_gltf((mo["hub"]["forward"], mo["hub"]["left"], prop[1]))
        assert abs(prop[0] - want[0]) < 1e-3 and abs(prop[2] - want[2]) < 1e-3, \
            "%s: motor %d's prop is not at its hub" % (name, mo["index"])
    fl = info["motors"][3]["hub"]
    assert fl["forward"] > 0 and fl["left"] > 0, "%s: motor 3 must be front left" % name


def centroid_of(mesh):
    pts = [to_gltf(p) for _, vs in mesh.tris for p, _, _ in vs]
    return [sum(p[k] for p in pts) / len(pts) for k in range(3)]


def main(argv):
    names = argv[1:] or list(MODELS)
    for name in names:
        parts, info = MODELS[name]()
        check(name, parts, info)
        out = os.path.join(HERE, name)
        os.makedirs(out, exist_ok=True)
        write_gltf(os.path.join(out, name + ".gltf"), name, parts)
        write_obj(os.path.join(out, name + ".obj"), name, parts)
        meta = {
            "name": name,
            "units": "metres",
            "axes": {"gltf_and_obj": {"forward": "+Z", "up": "+Y", "left": "+X", "right": "-X"},
                     "json_positions": "named: forward / left / up, metres from the origin"},
            "parts": {"body": "frame, stack, battery, camera, antenna, motors (always shown)",
                      "prop_still_<i>": "motor i's prop, standing still (below about 1500 rpm)",
                      "prop_spin_<i>": "motor i's prop, spinning: a translucent disc",
                      "rule": "per motor: show prop_spin_<i> or prop_still_<i> by frame.motors[i].rpm, "
                              "neither once frame.motors[i].prop_damage >= 1"},
            "materials": {m: {"rgba": list(c)} for m, c in COLORS.items()},
        }
        meta.update(info)
        with open(os.path.join(out, name + ".json"), "w") as f:
            json.dump(meta, f, indent=2)
            f.write("\n")
        tris = sum(len(m.tris) for m in parts.values())
        print("%s: %d triangles in %d parts -> %s" % (name, tris, len(parts), os.path.relpath(out)))


if __name__ == "__main__":
    sys.exit(main(sys.argv))
