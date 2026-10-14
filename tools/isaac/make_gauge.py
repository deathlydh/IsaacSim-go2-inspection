"""Procedural analog gauge (pressure gauge / dial thermometer) for our Isaac Sim scenes.

WHAT
    1. render_dial_png()  - draws the dial face (white face, black ticks and numbers, unit, tag label,
                            optional red zone) into a PNG with PIL. The needle is NOT painted on the texture.
    2. add_gauge()        - authors a complete gauge with pure pxr (no Kit / omni.* needed):
                            housing cylinder + bezel ring (dark metal), textured dial disc, a separate needle
                            under a pivot Xform rotated to the value, optional glass, a short connection stem,
                            a static collider on the housing only, and ground-truth attributes on the root.
    3. set_gauge_value()  - re-rotates the needle of an existing gauge and updates its attributes.
    4. gauge_ground_truth() - reads everything a CV pipeline needs (value, range, world center, face normal...).

WHY
    NVIDIA's asset library has no gauge with a movable needle (on the SimReady gauges the needle is painted
    into the texture). Building the gauge ourselves means the true reading is known exactly and can be changed
    per scene, which gives free labelled data for the gauge-reading CV model.

LOCAL FRAME CONVENTION (important for placing gauges)
    - Origin  = center of the dial face (the visible front surface).  -> "gauge center"
    - +X      = face normal: the dial looks along local +X (a viewer reads the gauge standing on the +X side).
    - +Z      = "12 o'clock" on the dial (up when the gauge is mounted upright).
    - +Y      = "3 o'clock" as seen by that viewer (viewer looks along -X, up is +Z, so right is +Y).
    - The housing extends BEHIND the face, from x = -depth to x = 0.
    So to make a gauge face world direction (cos a, sin a, 0), use rotate_xyz_deg=(0, 0, a in degrees).
    Examples: face +X -> (0,0,0); face +Y -> (0,0,90); face -X -> (0,0,180); face -Y -> (0,0,-90).

NEEDLE ANGLE CONVENTION
    needle_angle_deg = clockwise angle from 12 o'clock as seen from the front (viewer side).
    value = vmin -> -sweep/2 (for 270 deg: 7:30 position); value = vmax -> +sweep/2 (4:30 position).
    In USD the pivot gets xformOp:rotateX = -needle_angle_deg (a positive rotation about +X turns
    12 o'clock towards 9 o'clock, i.e. counter-clockwise for the viewer).

TEXTURE MAPPING
    The dial disc uses primvar "st" with the USD convention st=(0,0) at the image's bottom-left:
    u grows to the viewer's right (+Y), v grows up (+Z). The PNG is drawn upright (12 o'clock at the top).
    The texture path is written RELATIVE to the layer that is being edited, so a scene folder can be moved.

Ground-truth attributes on the gauge root prim (custom, namespace "inspection:"):
    inspection:type ("analog_gauge"), inspection:value, inspection:vmin, inspection:vmax, inspection:unit,
    inspection:label, inspection:sweepDeg, inspection:needleAngleDeg, inspection:diameter (m),
    inspection:faceNormalLocal (1,0,0), inspection:upLocal (0,0,1), inspection:dialTexture (asset).

How you would do the same by hand in the Isaac Sim GUI (for learning):
    Create > Shape > Cylinder (housing), Create > Mesh > Disk (dial), Create > Material > UsdPreviewSurface,
    plug an image into Diffuse Color; Create > Xform "NeedlePivot" with a thin Plane/Cube as the needle, then
    rotate the pivot about X; add attributes in Property panel > "+ Add" > Attribute.

Usage (inside Kit, e.g. from build_factory.py):
    import sys; sys.path.insert(0, "D:/go2-inspection/tools/isaac"); import make_gauge
    make_gauge.render_dial_png("D:/.../g1.png", 0, 10, "bar", "PI-101", major_step=1, minor_step=0.2, red_from=8)
    make_gauge.add_gauge(stage, "/World/Gauges/Gauge_01", dial_png="D:/.../g1.png", value=6.4, vmin=0, vmax=10,
                         diameter_m=0.1, translate=(0, 0, 0.5), rotate_xyz_deg=(0, 0, 90), label="PI-101", unit="bar")
    make_gauge.set_gauge_value(stage, "/World/Gauges/Gauge_01", 7.2)
"""

from __future__ import annotations

import math
import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

ATTR_NS = "inspection"
PIVOT_NAME = "NeedlePivot"

# Small offsets (meters) in front of the housing face, so the layers do not z-fight.
X_DIAL = 0.0008     # dial disc
X_NEEDLE = 0.0025   # needle plate
HUB_X0, HUB_X1 = 0.0020, 0.0050  # needle hub (cap) spans this x range


# ----------------------------------------------------------------------------------------------
# Math helpers
# ----------------------------------------------------------------------------------------------
def value_to_angle_deg(value: float, vmin: float, vmax: float, sweep_deg: float = 270.0) -> float:
    """Needle angle, degrees clockwise from 12 o'clock as seen from the front of the dial."""
    if vmax <= vmin:
        raise ValueError(f"vmax ({vmax}) must be greater than vmin ({vmin})")
    frac = (float(value) - vmin) / (vmax - vmin)
    return -sweep_deg / 2.0 + sweep_deg * frac


def _decimals_for(step: float) -> int:
    """How many decimals a scale label needs, e.g. 1 -> 0, 0.2 -> 1, 0.05 -> 2."""
    for d in range(0, 4):
        if abs(step * 10**d - round(step * 10**d)) < 1e-6:
            return d
    return 3


def _fmt(v: float, decimals: int) -> str:
    s = f"{v:.{decimals}f}"
    if s.startswith("-") and float(s) == 0.0:
        s = s[1:]  # avoid "-0"
    return s


# ----------------------------------------------------------------------------------------------
# 1. Dial texture
# ----------------------------------------------------------------------------------------------
def _load_font(px: int, bold: bool):
    """A TrueType font of the given pixel size. Windows Arial first, then DejaVu, then PIL's default."""
    from PIL import ImageFont

    names = ["arialbd.ttf", "DejaVuSans-Bold.ttf"] if bold else ["arial.ttf", "DejaVuSans.ttf"]
    for name in names:
        for candidate in (name, os.path.join(os.environ.get("WINDIR", "C:/Windows"), "Fonts", name)):
            try:
                return ImageFont.truetype(candidate, px)
            except OSError:
                pass
    try:
        return ImageFont.load_default(size=px)  # Pillow >= 10.1
    except TypeError:
        return ImageFont.load_default()


def render_dial_png(
    path: str,
    vmin: float,
    vmax: float,
    unit: str,
    label: str,
    major_step: float,
    minor_step: float,
    red_from: float | None = None,
    size_px: int = 1024,
    sweep_deg: float = 270.0,
) -> str:
    """Draw an analog dial face (no needle) into `path` (PNG, RGB, size_px x size_px). Returns `path`.

    The circle inscribed in the square image is exactly the visible dial disc of add_gauge().
    Layout: ticks at the rim, numbers inside the ticks at every major step, unit text below the pivot,
    the tag `label` below the unit (both sit in the free sector at the bottom), red band from `red_from`
    to vmax. White face + black print gives the best contrast for gauge-reading CV.
    """
    from PIL import Image, ImageDraw

    if vmax <= vmin:
        raise ValueError("vmax must be greater than vmin")
    if minor_step <= 0 or major_step <= 0:
        raise ValueError("steps must be positive")

    ss = 2  # draw at 2x and downsample: PIL polygons are not anti-aliased
    S = int(size_px) * ss
    c = S / 2.0
    R = S / 2.0  # image radius == dial disc radius
    img = Image.new("RGB", (S, S), (255, 255, 255))
    d = ImageDraw.Draw(img)

    def pt(theta_deg: float, r: float):
        t = math.radians(theta_deg)
        return (c + r * math.sin(t), c - r * math.cos(t))  # clockwise from 12 o'clock, image y down

    r_out = 0.90 * R          # outer end of the ticks
    major_len, minor_len = 0.12 * R, 0.06 * R
    # Thick enough for the 720p robot camera at ~0.9 m (~1 px per mm on a 100 mm dial).
    major_w, minor_w = 0.032 * R, 0.014 * R

    # Thin grey ring near the edge (printed border of the dial).
    d.ellipse([c - 0.975 * R, c - 0.975 * R, c + 0.975 * R, c + 0.975 * R], outline=(150, 150, 150), width=max(2, int(0.006 * R)))

    # Red zone band (under the ticks).
    if red_from is not None and vmin <= red_from < vmax:
        a0 = value_to_angle_deg(red_from, vmin, vmax, sweep_deg)
        a1 = sweep_deg / 2.0
        band = 0.055 * R
        box = [c - r_out, c - r_out, c + r_out, c + r_out]
        d.arc(box, start=a0 - 90.0, end=a1 - 90.0, fill=(210, 20, 20), width=int(band))  # PIL: 0 deg = 3 o'clock, clockwise

    # Scale arc along the outer tick ends.
    box = [c - r_out, c - r_out, c + r_out, c + r_out]
    d.arc(box, start=-sweep_deg / 2.0 - 90.0, end=sweep_deg / 2.0 - 90.0, fill=(0, 0, 0), width=max(2, int(0.006 * R)))

    # Ticks: count in integer minor steps to avoid float drift.
    n_minor = int(round((vmax - vmin) / minor_step))
    ratio = major_step / minor_step
    major_every = int(round(ratio))
    if abs(ratio - major_every) > 1e-6 or major_every < 1:
        raise ValueError("major_step must be an integer multiple of minor_step")
    decimals = _decimals_for(major_step)
    numbers = []
    for i in range(n_minor + 1):
        v = vmin + i * minor_step
        theta = value_to_angle_deg(v, vmin, vmax, sweep_deg)
        is_major = i % major_every == 0
        length, width = (major_len, major_w) if is_major else (minor_len, minor_w)
        # Tick as a rotated rectangle: two points along the radius, offset sideways by width/2.
        t = math.radians(theta)
        ux, uy = math.sin(t), -math.cos(t)       # radial unit vector (image coords)
        px, py = -uy, ux                          # perpendicular
        r0, r1 = r_out - length, r_out
        hw = width / 2.0
        poly = [
            (c + ux * r0 + px * hw, c + uy * r0 + py * hw),
            (c + ux * r1 + px * hw, c + uy * r1 + py * hw),
            (c + ux * r1 - px * hw, c + uy * r1 - py * hw),
            (c + ux * r0 - px * hw, c + uy * r0 - py * hw),
        ]
        d.polygon(poly, fill=(0, 0, 0))
        if is_major:
            numbers.append((theta, _fmt(v, decimals)))

    # Numbers inside the ticks; shrink the font for long labels like "-0.8" or "160".
    longest = max(len(s) for _, s in numbers)
    num_px = int((0.135 if longest <= 2 else 0.115 if longest == 3 else 0.10) * R)
    num_font = _load_font(num_px, bold=True)
    r_num = r_out - major_len - 0.13 * R
    for theta, s in numbers:
        d.text(pt(theta, r_num), s, fill=(0, 0, 0), font=num_font, anchor="mm")

    # Unit (big) and tag label (smaller) in the free bottom sector.
    if unit:
        d.text((c, c + 0.30 * R), unit, fill=(0, 0, 0), font=_load_font(int(0.13 * R), bold=True), anchor="mm")
    if label:
        d.text((c, c + 0.52 * R), label, fill=(40, 40, 40), font=_load_font(int(0.085 * R), bold=False), anchor="mm")

    img = img.resize((int(size_px), int(size_px)), Image.LANCZOS)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    img.save(path, format="PNG", optimize=True)
    return path


# ----------------------------------------------------------------------------------------------
# 2. USD authoring helpers
# ----------------------------------------------------------------------------------------------
def _preview_material(stage: Usd.Stage, path: str, color, metallic=0.0, roughness=0.5, opacity=1.0) -> UsdShade.Material:
    """Plain UsdPreviewSurface material (no texture)."""
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    if opacity < 1.0:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
        sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.5)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _texture_asset_path(stage: Usd.Stage, file_path: str) -> str:
    """Path to write into the USD file: relative to the edited layer's folder when possible."""
    file_path = os.path.abspath(file_path)
    layer = stage.GetEditTarget().GetLayer()
    real = layer.realPath if layer is not None else ""
    if real:
        try:
            rel = os.path.relpath(file_path, os.path.dirname(real)).replace("\\", "/")
            return rel if rel.startswith(".") else "./" + rel
        except ValueError:  # different drive letters on Windows
            pass
    return file_path.replace("\\", "/")


def _textured_material(stage: Usd.Stage, path: str, texture_asset: str) -> UsdShade.Material:
    """UsdPreviewSurface whose diffuseColor comes from a UsdUVTexture reading primvar 'st'."""
    mat = UsdShade.Material.Define(stage, path)
    surf = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)  # matte printed face, few highlights
    surf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)

    reader = UsdShade.Shader.Define(stage, f"{path}/StReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    # String per the UsdPreviewSurface spec (a Token may be ignored by RTX -> blank dial).
    reader.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    tex = UsdShade.Shader.Define(stage, f"{path}/DialTexture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture_asset))
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    return mat


def _bind(prim: Usd.Prim, mat: UsdShade.Material) -> None:
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def _mesh(stage, path, points, counts, indices, normals=None, st=None, double_sided=False) -> UsdGeom.Mesh:
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)  # keep it polygonal (default would smooth it)
    m.CreateDoubleSidedAttr(bool(double_sided))
    if normals is not None:
        m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*n) for n in normals]))
        m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    if st is not None:
        pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(*uv) for uv in st]))
    m.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(m.GetPointsAttr().Get()))
    return m


def _disc_x(stage, path, radius, x, segments=128, with_uv=False) -> UsdGeom.Mesh:
    """Flat disc in the plane x = const, facing +X (triangle fan, counter-clockwise seen from +X)."""
    pts = [(x, 0.0, 0.0)]
    st = [(0.5, 0.5)]
    for i in range(segments):
        phi = 2.0 * math.pi * i / segments  # from +Y towards +Z
        y, z = radius * math.cos(phi), radius * math.sin(phi)
        pts.append((x, y, z))
        st.append((0.5 + 0.5 * y / radius, 0.5 + 0.5 * z / radius))
    counts, idx = [], []
    for i in range(segments):
        counts.append(3)
        idx += [0, 1 + i, 1 + (i + 1) % segments]
    normals = [(1.0, 0.0, 0.0)] * len(pts)
    return _mesh(stage, path, pts, counts, idx, normals=normals, st=st if with_uv else None)


def _bezel_x(stage, path, r_in, r_out, x0, x1, segments=96) -> UsdGeom.Mesh:
    """Ring that sticks out in front of the housing: front annulus + inner and outer walls."""
    pts = []
    for i in range(segments):
        phi = 2.0 * math.pi * i / segments
        cy, sz = math.cos(phi), math.sin(phi)
        pts += [
            (x1, r_in * cy, r_in * sz),   # 4*i + 0  front inner
            (x1, r_out * cy, r_out * sz),  # 4*i + 1  front outer
            (x0, r_in * cy, r_in * sz),   # 4*i + 2  back inner
            (x0, r_out * cy, r_out * sz),  # 4*i + 3  back outer
        ]
    counts, idx = [], []
    for i in range(segments):
        a, b = 4 * i, 4 * ((i + 1) % segments)
        # front face (normal +X): inner(i) -> outer(i) -> outer(i+1) -> inner(i+1)
        counts.append(4); idx += [a + 0, a + 1, b + 1, b + 0]
        # outer wall (normal outward)
        counts.append(4); idx += [a + 1, a + 3, b + 3, b + 1]
        # inner wall (normal towards the axis)
        counts.append(4); idx += [a + 0, b + 0, b + 2, a + 2]
    return _mesh(stage, path, pts, counts, idx)


def _cylinder(stage, path, axis, radius, height, center) -> UsdGeom.Cylinder:
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateAxisAttr(axis)
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateHeightAttr(float(height))
    h, r = height / 2.0, radius
    ext = {"X": ((-h, -r, -r), (h, r, r)), "Y": ((-r, -h, -r), (r, h, r)), "Z": ((-r, -r, -h), (r, r, h))}[axis]
    cyl.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*ext[0]), Gf.Vec3f(*ext[1])]))
    UsdGeom.XformCommonAPI(cyl).SetTranslate(Gf.Vec3d(*center))
    return cyl


def _set_attr(prim: Usd.Prim, name: str, type_name, value) -> None:
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, type_name, custom=True)
    attr.Set(value)


# ----------------------------------------------------------------------------------------------
# 3. The gauge
# ----------------------------------------------------------------------------------------------
def add_gauge(
    stage: Usd.Stage,
    prim_path: str,
    *,
    dial_png: str,
    value: float,
    vmin: float,
    vmax: float,
    diameter_m: float,
    sweep_deg: float = 270.0,
    translate=(0.0, 0.0, 0.0),
    rotate_xyz_deg=(0.0, 0.0, 0.0),
    unit: str = "",
    label: str = "",
    mount: str = "bottom",
    stem_length_m: float = 0.03,
    depth_m: float | None = None,
    add_glass: bool = False,
    collider: bool = True,
) -> Usd.Prim:
    """Author a gauge at `prim_path`. Returns the root prim.

    translate        world (or parent-relative) position of the dial FACE CENTER, meters.
    rotate_xyz_deg   rotation of the gauge root, degrees (XYZ order). Face normal = local +X (see module doc).
    mount            "bottom": connection stem goes down from the housing (gauge sitting on top of a pipe);
                     "back":   stem goes backwards (-X) from the housing (gauge on a wall / tank / thermowell);
                     "none":   no stem.
    stem_length_m    visible stem length; the stem also sinks 5 mm into the housing.
    depth_m          housing depth (default 0.30 * diameter).
    add_glass        thin, mostly transparent cover. Off by default: reflections on glass make reading harder.
    collider         static collider (no rigid body) on the housing cylinder only.
    """
    if not (vmin <= value <= vmax):
        raise ValueError(f"{prim_path}: value {value} outside [{vmin}, {vmax}]")
    R = diameter_m / 2.0                 # housing radius
    r_dial = 0.90 * R                    # visible dial radius (texture circle)
    depth = depth_m if depth_m is not None else 0.30 * diameter_m
    bezel_h = max(0.006, 0.08 * R)       # bezel sticks out in front of the face

    root = UsdGeom.Xform.Define(stage, prim_path)
    xf = UsdGeom.XformCommonAPI(root)
    xf.SetTranslate(Gf.Vec3d(*translate))
    xf.SetRotate(Gf.Vec3f(*rotate_xyz_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    # Materials live inside the gauge so the gauge is self-contained (easy to copy / delete).
    looks = f"{prim_path}/Looks"
    m_case = _preview_material(stage, f"{looks}/DarkMetal", (0.10, 0.10, 0.11), metallic=0.85, roughness=0.35)
    m_needle = _preview_material(stage, f"{looks}/NeedleBlack", (0.02, 0.02, 0.02), metallic=0.0, roughness=0.45)
    m_brass = _preview_material(stage, f"{looks}/Brass", (0.72, 0.53, 0.22), metallic=1.0, roughness=0.35)
    m_dial = _textured_material(stage, f"{looks}/Dial", _texture_asset_path(stage, dial_png))

    # Housing: cylinder along X, behind the face (x in [-depth, 0]).
    housing = _cylinder(stage, f"{prim_path}/Housing", "X", R, depth, (-depth / 2.0, 0.0, 0.0))
    _bind(housing.GetPrim(), m_case)
    if collider:
        UsdPhysics.CollisionAPI.Apply(housing.GetPrim())  # no RigidBodyAPI -> static collider

    bezel = _bezel_x(stage, f"{prim_path}/Bezel", r_dial, R, -0.001, bezel_h)
    _bind(bezel.GetPrim(), m_case)

    dial = _disc_x(stage, f"{prim_path}/Dial", r_dial, X_DIAL, with_uv=True)
    _bind(dial.GetPrim(), m_dial)

    # Needle: flat 5-gon pointing to +Z (12 o'clock) in the pivot frame; the pivot rotates about X.
    pivot = UsdGeom.Xform.Define(stage, f"{prim_path}/{PIVOT_NAME}")
    pivot.AddRotateXOp(UsdGeom.XformOp.PrecisionDouble).Set(0.0)  # double: stored angle == ground truth exactly
    L, T = 0.84 * r_dial, 0.20 * r_dial          # tip length, tail length
    # Half widths at the hub, tail and tip. Sized for the 720p robot camera at ~0.9 m
    # (~1 px per mm): on a 100 mm dial the needle is ~4.5 px wide at the hub and ~1.8 px at the tip.
    wh, wt = 0.05 * r_dial, 0.030 * r_dial
    tip_w = 0.02 * r_dial
    needle_pts = [
        (X_NEEDLE, wh, 0.0), (X_NEEDLE, tip_w, L), (X_NEEDLE, -tip_w, L),
        (X_NEEDLE, -wh, 0.0), (X_NEEDLE, -wt, -T), (X_NEEDLE, wt, -T),
    ]  # counter-clockwise seen from +X -> normal +X
    needle = _mesh(stage, f"{prim_path}/{PIVOT_NAME}/Needle", needle_pts, [6], [0, 1, 2, 3, 4, 5],
                   normals=[(1.0, 0.0, 0.0)] * 6, double_sided=True)
    _bind(needle.GetPrim(), m_needle)
    hub = _cylinder(stage, f"{prim_path}/{PIVOT_NAME}/Hub", "X", 0.075 * r_dial, HUB_X1 - HUB_X0,
                    ((HUB_X0 + HUB_X1) / 2.0, 0.0, 0.0))
    _bind(hub.GetPrim(), m_needle)

    if add_glass:
        m_glass = _preview_material(stage, f"{looks}/Glass", (0.9, 0.95, 1.0), metallic=0.0, roughness=0.05, opacity=0.12)
        glass = _disc_x(stage, f"{prim_path}/Glass", r_dial, bezel_h - 0.0005)
        _bind(glass.GetPrim(), m_glass)

    # Connection stem (process connection), brass.
    stem_r = min(0.012, 0.09 * diameter_m)
    if mount == "bottom":
        h = stem_length_m + 0.005
        stem = _cylinder(stage, f"{prim_path}/Stem", "Z", stem_r, h, (-depth / 2.0, 0.0, -R - stem_length_m / 2.0 + 0.0025))
        _bind(stem.GetPrim(), m_brass)
    elif mount == "back":
        h = stem_length_m + 0.005
        stem = _cylinder(stage, f"{prim_path}/Stem", "X", stem_r, h, (-depth - stem_length_m / 2.0 + 0.0025, 0.0, 0.0))
        _bind(stem.GetPrim(), m_brass)
    elif mount != "none":
        raise ValueError(f"mount must be 'bottom', 'back' or 'none', got {mount!r}")

    # Ground truth for CV, stored on the root.
    p = root.GetPrim()
    _set_attr(p, f"{ATTR_NS}:type", Sdf.ValueTypeNames.String, "analog_gauge")
    _set_attr(p, f"{ATTR_NS}:vmin", Sdf.ValueTypeNames.Double, float(vmin))
    _set_attr(p, f"{ATTR_NS}:vmax", Sdf.ValueTypeNames.Double, float(vmax))
    _set_attr(p, f"{ATTR_NS}:unit", Sdf.ValueTypeNames.String, unit)
    _set_attr(p, f"{ATTR_NS}:label", Sdf.ValueTypeNames.String, label)
    _set_attr(p, f"{ATTR_NS}:sweepDeg", Sdf.ValueTypeNames.Double, float(sweep_deg))
    _set_attr(p, f"{ATTR_NS}:diameter", Sdf.ValueTypeNames.Double, float(diameter_m))
    _set_attr(p, f"{ATTR_NS}:faceNormalLocal", Sdf.ValueTypeNames.Float3, Gf.Vec3f(1.0, 0.0, 0.0))
    _set_attr(p, f"{ATTR_NS}:upLocal", Sdf.ValueTypeNames.Float3, Gf.Vec3f(0.0, 0.0, 1.0))
    _set_attr(p, f"{ATTR_NS}:dialTexture", Sdf.ValueTypeNames.Asset, Sdf.AssetPath(_texture_asset_path(stage, dial_png)))
    set_gauge_value(stage, prim_path, value)
    return p


def set_gauge_value(stage: Usd.Stage, prim_path: str, value: float) -> float:
    """Rotate the needle of the gauge at `prim_path` to `value` and update its attributes.

    Works on a scene file (in-memory stage) and on the live GUI stage alike. Returns the needle angle
    (degrees clockwise from 12 o'clock, seen from the front). Raises ValueError outside [vmin, vmax].
    """
    root = stage.GetPrimAtPath(prim_path)
    if not root or not root.IsValid():
        raise ValueError(f"no gauge at {prim_path}")
    vmin = root.GetAttribute(f"{ATTR_NS}:vmin").Get()
    vmax = root.GetAttribute(f"{ATTR_NS}:vmax").Get()
    sweep = root.GetAttribute(f"{ATTR_NS}:sweepDeg").Get()
    if vmin is None or vmax is None or sweep is None:
        raise ValueError(f"{prim_path} is not a gauge made by make_gauge.add_gauge")
    if not (vmin <= value <= vmax):
        raise ValueError(f"{prim_path}: value {value} outside [{vmin}, {vmax}]")
    angle = value_to_angle_deg(value, vmin, vmax, sweep)
    pivot = UsdGeom.Xformable(stage.GetPrimAtPath(f"{prim_path}/{PIVOT_NAME}"))
    ops = [op for op in pivot.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeRotateX]
    op = ops[0] if ops else pivot.AddRotateXOp(UsdGeom.XformOp.PrecisionDouble)
    op.Set(-angle)  # +rotateX is counter-clockwise for the viewer, our angle is clockwise
    _set_attr(root, f"{ATTR_NS}:value", Sdf.ValueTypeNames.Double, float(value))
    _set_attr(root, f"{ATTR_NS}:needleAngleDeg", Sdf.ValueTypeNames.Double, float(angle))
    return angle


def gauge_ground_truth(stage: Usd.Stage, prim_path: str, time=Usd.TimeCode.Default()) -> dict:
    """Everything a CV check needs about one gauge, in world coordinates (meters)."""
    root = stage.GetPrimAtPath(prim_path)
    if not root or not root.IsValid():
        raise ValueError(f"no gauge at {prim_path}")
    get = lambda n: root.GetAttribute(f"{ATTR_NS}:{n}").Get()  # noqa: E731
    m = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(time)
    center = m.Transform(Gf.Vec3d(0.0, 0.0, 0.0))
    normal = m.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0)).GetNormalized()
    up = m.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)).GetNormalized()
    right = m.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0)).GetNormalized()
    tex = get("dialTexture")
    r4 = lambda v: [round(c, 4) + 0.0 for c in v]  # noqa: E731  (+ 0.0 turns -0.0 into 0.0)
    return {
        "prim_path": prim_path,
        "label": get("label"),
        "value": get("value"),
        "vmin": get("vmin"),
        "vmax": get("vmax"),
        "unit": get("unit"),
        "sweep_deg": get("sweepDeg"),
        "needle_angle_deg": get("needleAngleDeg"),
        "diameter_m": get("diameter"),
        "center_world": r4(center),
        "face_normal_world": r4(normal),
        "up_world": r4(up),
        "right_world": r4(right),
        "dial_texture": tex.path if tex else None,
    }
