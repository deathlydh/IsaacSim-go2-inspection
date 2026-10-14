"""Build the factory hall scene file: scenes/factory_hall.usda (+ gauge dial PNGs + meta JSON).

WHAT
    - Writes D:/go2-inspection/scenes/factory_hall.usda with Usd.Stage.CreateNew on an IN-MEMORY stage.
      The stage that is open in the Isaac Sim window is never touched (no open/new/save_as there), so this
      script is safe to run while a simulation is playing.
    - /PhysicsScene: same settings as build_scene.py (CPU solver, MBP, TGS, 200 steps/s, gravity -Z).
    - /World/Env: a REFERENCE to NVIDIA's Simple_Warehouse/warehouse.usd (full cloud URL). Racks, pallets,
      boxes, bins, pushcart and rack signs are deactivated (overrides only, the NVIDIA file is not changed).
      Walls, columns, beams, ceiling, lamps/lights, floor, floor markings, fuse boxes and wall signs stay.
    - /World/Props: two process lines + a utility corner built from NVIDIA Office pipes/valve and SimReady
      containers/barriers, plus simple procedural pipe supports, flanges and an instrument stand.
    - /World/Gauges: 5 procedural gauges (tools/isaac/make_gauge.py) with different scales and values.
    - scenes/factory_hall_meta.json: robot spawn pose, per-gauge ground truth and inspection waypoints.

WHY
    start_go2.py opens a scene file and adds the robot at runtime. Keeping the hall as a small .usda that only
    REFERENCES cloud assets keeps the repo light and the scene reproducible: re-run this script after edits.

LAYOUT (meters, Z up, floor z = 0, hall interior about x -10..9.4, y -12.2..18)
    Aisle along +X on the centerline y = 0 (clear width about 3.2 m, x from -9 to 8.5).
    Line A (south side, y about -2.3..-2.9): steam header DN125 (SM_PipeBig_3m x3) at z = 0.32 with valve, a
           small pipe above it, supports and flanges. Gauges PI-101, PI-102 (on top) and TI-103 (side).
    Line B (north side, y about 2.4..3.8): stainless IBC (gauge PI-201 on its wall), caged IBC with a vacuum
           gauge PI-202 on a stand, steel drums.
    Utility: electrical fuse-box cabinet behind a rail guard, a stored pipe bundle, rail guards at the
           line ends. All gauge faces point at the aisle.

PHYSICS NOTES
    SimReady assets ship with Physics variant 'physics' (a dynamic rigid body) and SDF mesh colliders.
    We select variant 'none', disable any remaining rigid body (physics:rigidBodyEnabled = false -> static),
    and override physics:approximation (convexHull for tanks/drums, none = triangle mesh for pipes and
    rail guards) because SDF colliders are a GPU feature [CPU support not verified]. Static bodies may
    use exact triangle meshes; convexDecomposition (V-HACD) on the 62k-point rail guard stalled first Play.
    Their colliders live inside instanced prims, which cannot be edited, so those prims are de-instanced
    (instanceable = false) first. Office pipes / fuse box already have static triangle-mesh colliders.
    The warehouse floor (meshes + its CollisionPlane) gets the same friction material as build_scene.py.

HOW TO RUN (Isaac Sim must be running with the python server):
    python -I D:\\isaacsim\\skills\\isaac-sim-remote\\scripts\\isaacsim_send.py --timeout 300 --file D:\\go2-inspection\\tools\\isaac\\build_factory.py
    Then start the robot in it:  start_go2.py with --arg "scene_path='D:/go2-inspection/scenes/factory_hall.usda'"

Injected args (isaacsim_send.py --arg), all optional:
    project_root  (default D:/go2-inspection)
    scene_path    (default <project_root>/scenes/factory_hall.usda)
    meta_path     (default <project_root>/scenes/factory_hall_meta.json)
    texture_dir   (default <project_root>/scenes/textures/gauges)

How you would do it by hand in the GUI (for learning):
    Content browser > Isaac > Environments > Simple_Warehouse > drag warehouse.usd onto /World (creates a
    reference); select a rack > right click > Deactivate; drag props from Isaac > SimReady; select a prop >
    Property > Variants > Physics = none; select its mesh > Property > Physics > Collider > Approximation.
"""

import importlib.util
import json
import math
import os
import re
import sys

import omni.client
import omni.usd
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

if "project_root" not in dir():
    project_root = "D:/go2-inspection"
if "scene_path" not in dir():
    scene_path = f"{project_root}/scenes/factory_hall.usda"
if "meta_path" not in dir():
    meta_path = f"{project_root}/scenes/factory_hall_meta.json"
if "texture_dir" not in dir():
    texture_dir = f"{project_root}/scenes/textures/gauges"

_tools_dir = f"{project_root}/tools/isaac"
if _tools_dir not in sys.path:
    sys.path.insert(0, _tools_dir)


def _load_fresh(name, path):
    """Import `path` as module `name`, compiling the CURRENT source text.

    Isaac Sim stays open between runs, so a plain `import` keeps the old code. importlib.reload was observed
    to return stale code once in Kit (cached bytecode), so we compile the file text ourselves.
    """
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    with open(path, encoding="utf-8") as f:
        exec(compile(f.read(), path, "exec"), module.__dict__)
    return module


make_gauge = _load_fresh("make_gauge", f"{_tools_dir}/make_gauge.py")

# ------------------------------------------------------------------------------------------------
# Assets (paths relative to the Isaac assets root; every one is checked with omni.client.stat)
# ------------------------------------------------------------------------------------------------
WAREHOUSE = "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
SR = "/Isaac/SimReady/Industrial/Warehouse"
ASSETS = {
    "pipe_big_3m": "/Isaac/Environments/Office/Props/SM_PipeBig_3m.usd",       # 3 m along +Y, r = 0.065
    "pipe_small_3m": "/Isaac/Environments/Office/Props/SM_PipeSmall_3m.usd",   # 3 m along +Y, r = 0.040
    "valve_big": "/Isaac/Environments/Office/Props/SM_ValveBig.usd",           # sits on a big pipe along Y
    "fuse_box": "/Isaac/Environments/Simple_Warehouse/Props/SM_FuseBox_01.usd",  # cabinet, front = local +X
    "ibc_steel": f"{SR}/Containers/Ibc_Tank_Steel_Stainless_A01/sm_ibcTank_steel_stainless_a01_01.usd",
    "ibc_cage": f"{SR}/Containers/Ibc_Tank_A02/sm_ibcTank_a02_01.usd",
    "drum_a01": f"{SR}/Containers/Drum_Steel_A01/sm_drum_steel_a01_01.usd",
    "drum_a02": f"{SR}/Containers/Drum_Steel_A02/sm_drum_steel_a02_01.usd",
    "pipe_bundle": f"{SR}/Equipment/Pipe_Plastic_Gray_B08/sm_pipe_plastic_gray_b08_01.usd",  # 7 m along Y
    "rail_guard": f"{SR}/Barriers/Rail_Guard_A09/sm_rail_guard_a09_01.usd",  # 2.79 m along X, 1 m tall
}
# Note: the plan suggested SimReady Electrical_Main_Terminal_A01 as the cabinet, but it is a console on a
# base plate (1.7 x 3.6 m, 12 x 4K textures), not a cabinet. The warehouse's own SM_FuseBox_01 is used instead.

# Top-level children of the warehouse that are removed (name prefix -> why).
DEACTIVATE_PREFIXES = {
    "SM_RackFrame": "storage rack frame",
    "SM_RackShelf": "storage rack shelf",
    "SM_Rackshield": "rack foot guard",
    "SM_PaletteA": "pallets (on the floor and on shelves)",
    "SM_CardBoxA": "cardboard box",
    "KLT_Bins": "plastic bins group",
    "SM_SignCVer": "rack labels (would float without racks)",
    "S_AisleSign": "rack aisle signs (would float without racks)",
    "S_Barcode": "rack barcode label",
    "SM_PushcartA": "pushcart (clutter)",
    "SM_FireExtinguisher": "hang on rack frames (would float without racks)",
}

# ------------------------------------------------------------------------------------------------
# Layout constants (see module docstring)
# ------------------------------------------------------------------------------------------------
HEADER_Y, HEADER_Z, HEADER_R = -2.30, 0.32, 0.065   # Line A main pipe axis and radius
SMALL_Y, SMALL_Z, SMALL_R = -2.75, 0.75, 0.040      # Line A small pipe
IBC_STEEL_XY = (-4.6, 3.05)
IBC_STEEL_WALL_HALF_Y = 0.531   # measured: stainless IBC side wall is at |y| = 0.531 m (frame corners 0.556)
STAND_XY = (-1.25, 2.30)        # instrument stand for the vacuum gauge
STAND_GAUGE_Z = 0.45            # vacuum gauge center height on that stand
AISLE = {"axis": "x", "centerline_y": 0.0, "x_range": [-9.0, 8.5]}
ROBOT_SPAWN = {"position": [-8.0, 0.0, 0.0], "yaw_deg": 0.0}
# Waypoint = gauge center projected on the floor + STANDOFF along the face normal. The Go2 front camera sits
# ~0.33 m ahead of the base, so the camera is ~0.87 m from the dial: a 100 mm dial is ~100 px wide at 720p and
# the whole dial stays inside the 45 deg vertical FOV even with the current crouched gait (camera ~0.25 m high).
WAYPOINT_STANDOFF_M = 1.2
ROBOT_HALF_LENGTH_M = 0.35      # Go2 body ~0.70 x 0.31 m; used for clearance checks
SPAWN_MIN_CLEARANCE_M = 1.0

# Gauges. depth = 0.30 * diameter (make_gauge default). Center = dial face center.
def _gauges():
    def depth(d):
        return 0.30 * d

    header_top = HEADER_Z + HEADER_R
    ibc_wall_y = IBC_STEEL_XY[1] - IBC_STEEL_WALL_HALF_Y
    g = [
        dict(id="Gauge_01", label="PI-101", unit="bar", vmin=0.0, vmax=10.0, value=6.4, major=1.0, minor=0.2,
             red_from=8.0, d=0.100, mount="bottom", stem=0.04, rot=90.0,
             center=(-4.6, HEADER_Y + depth(0.100) / 2, header_top + 0.04 + 0.050), on="Line A header (top)"),
        dict(id="Gauge_02", label="PI-102", unit="bar", vmin=0.0, vmax=16.0, value=11.5, major=2.0, minor=0.5,
             red_from=14.0, d=0.160, mount="bottom", stem=0.03, rot=90.0,
             center=(0.9, HEADER_Y + depth(0.160) / 2, header_top + 0.03 + 0.080), on="Line A header after valve (top)"),
        dict(id="Gauge_03", label="PI-201", unit="psi", vmin=0.0, vmax=160.0, value=87.0, major=20.0, minor=5.0,
             red_from=140.0, d=0.125, mount="back", stem=0.06, rot=-90.0,
             center=(IBC_STEEL_XY[0], ibc_wall_y - 0.05 - depth(0.125), 0.50), on="stainless IBC wall"),
        dict(id="Gauge_04", label="PI-202", unit="bar", vmin=-1.0, vmax=0.0, value=-0.62, major=0.2, minor=0.05,
             red_from=None, d=0.100, mount="bottom", stem=0.03, rot=-90.0,
             center=(STAND_XY[0], STAND_XY[1] - depth(0.100) / 2, STAND_GAUGE_Z), on="instrument stand by caged IBC"),
        dict(id="Gauge_05", label="TI-103", unit="\u00b0C", vmin=0.0, vmax=250.0, value=168.0, major=50.0, minor=10.0,
             red_from=200.0, d=0.100, mount="back", stem=0.06, rot=90.0,
             center=(2.0, HEADER_Y + HEADER_R + 0.05 + depth(0.100), HEADER_Z), on="Line A header (side thermowell)"),
    ]
    return g


# ------------------------------------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------------------------------------
def check_assets(root, rels):
    """Fail early with a clear message if an asset path does not exist in the library."""
    missing = []
    for rel in rels:
        result, _ = omni.client.stat(f"{root}{rel}")
        if result != omni.client.Result.OK:
            missing.append(f"{rel} ({result})")
    if missing:
        raise RuntimeError("missing assets:\n  " + "\n  ".join(missing))


def open_target_stage(path):
    """In-memory stage whose root layer is `path` (created fresh). Refuses to touch the GUI stage."""
    gui_stage = omni.usd.get_context().get_stage()
    norm = lambda p: os.path.normcase(os.path.abspath(p)) if p and "://" not in p else p  # noqa: E731
    if gui_stage is not None and norm(gui_stage.GetRootLayer().realPath) == norm(path):
        raise RuntimeError(f"{path} is open in the Isaac Sim window - open another scene first")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    existing = Sdf.Layer.Find(path)  # still cached from an earlier run in this Kit session
    if existing is not None:
        existing.Clear()
        return Usd.Stage.Open(existing)
    return Usd.Stage.CreateNew(path)


def define_physics_scene(stage):
    """Identical to build_scene.py: 200 Hz (policy training rate), CPU solver, MBP broadphase, TGS."""
    scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    physx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx.CreateTimeStepsPerSecondAttr().Set(200)
    physx.CreateSolverTypeAttr().Set("TGS")
    physx.CreateEnableGPUDynamicsAttr().Set(False)
    physx.CreateBroadphaseTypeAttr().Set("MBP")


def preview_material(stage, path, color, metallic=0.0, roughness=0.5):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def add_box(stage, path, center, size, mat, collider=True):
    """Axis-aligned box (UsdGeom.Cube scaled) with an optional static collider."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([Gf.Vec3f(-0.5), Gf.Vec3f(0.5)])
    xf = UsdGeom.XformCommonAPI(cube)
    xf.SetTranslate(Gf.Vec3d(*center))
    xf.SetScale(Gf.Vec3f(*size))
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat)
    if collider:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube.GetPrim()


def add_cylinder(stage, path, center, axis, radius, height, mat, collider=True):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateAxisAttr(axis)
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateHeightAttr(float(height))
    h, r = height / 2.0, radius
    ext = {"X": ((-h, -r, -r), (h, r, r)), "Y": ((-r, -h, -r), (r, h, r)), "Z": ((-r, -r, -h), (r, r, h))}[axis]
    cyl.CreateExtentAttr([Gf.Vec3f(*ext[0]), Gf.Vec3f(*ext[1])])
    UsdGeom.XformCommonAPI(cyl).SetTranslate(Gf.Vec3d(*center))
    UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(mat)
    if collider:
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    return cyl.GetPrim()


def make_static_colliders(stage, prim, approximation):
    """Make every collider under `prim` static and (optionally) set its mesh approximation.

    1. De-instance: colliders inside an instance are read-only "instance proxies"; setting
       instanceable = false on the outermost instance above them makes them editable (repeat for nesting).
    2. physics:rigidBodyEnabled = false on every RigidBodyAPI prim -> PhysX treats the body as static.
    3. physics:approximation = `approximation` on every mesh collider (None = keep the asset's value).
    Returns {"colliders": n, "approximations": {...}, "rigid_bodies_disabled": n, "deinstanced": n}.
    """
    deinstanced = 0
    for _ in range(8):  # nesting depth guard
        targets = set()
        for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
            if p.IsInstanceProxy() and (p.HasAPI(UsdPhysics.CollisionAPI) or p.HasAPI(UsdPhysics.RigidBodyAPI)):
                outer, a = None, p.GetParent()
                while a and a != prim.GetParent():
                    if a.IsInstance() and not a.IsInstanceProxy():
                        outer = a
                    a = a.GetParent()
                if outer is not None:
                    targets.add(outer.GetPath())
        if not targets:
            break
        for path in targets:
            stage.GetPrimAtPath(path).SetInstanceable(False)
            deinstanced += 1
    n_coll, n_rb, approx = 0, 0, {}
    for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if p.IsInstanceProxy():
            continue
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr().Set(False)
            n_rb += 1
        if p.HasAPI(UsdPhysics.CollisionAPI):
            n_coll += 1
            if approximation is not None and p.IsA(UsdGeom.Mesh):
                UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set(approximation)
            a = UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() if p.HasAPI(UsdPhysics.MeshCollisionAPI) else "(shape)"
            approx[a] = approx.get(a, 0) + 1
    return {"colliders": n_coll, "approximations": approx, "rigid_bodies_disabled": n_rb, "deinstanced": deinstanced}


def add_asset(stage, path, url, translate, rot_z_deg=0.0, approximation=None):
    """Reference an asset at `path`, place it, pick Physics variant 'none' and make its colliders static."""
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    prim.GetReferences().AddReference(url)
    xf = UsdGeom.XformCommonAPI(prim)
    xf.SetTranslate(Gf.Vec3d(*translate))
    xf.SetRotate(Gf.Vec3f(0.0, 0.0, float(rot_z_deg)), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    variant = None
    vsets = prim.GetVariantSets()
    if vsets.HasVariantSet("Physics"):
        vs = vsets.GetVariantSet("Physics")
        if "none" in vs.GetVariantNames():
            vs.SetVariantSelection("none")  # no dynamic rigid body
            variant = "none"
    info = make_static_colliders(stage, prim, approximation)
    info.update({"prim_path": path, "asset_url": url, "physics_variant": variant})
    return info


def rect_distance(px, py, rmin, rmax):
    """2D distance from a point to an axis-aligned rectangle (0 if inside)."""
    dx = max(rmin[0] - px, 0.0, px - rmax[0])
    dy = max(rmin[1] - py, 0.0, py - rmax[1])
    return math.hypot(dx, dy)


# ------------------------------------------------------------------------------------------------
# Build
# ------------------------------------------------------------------------------------------------
assets_root = get_assets_root_path()
if not assets_root:
    raise RuntimeError("Isaac assets root not found (get_assets_root_path() returned nothing)")
check_assets(assets_root, [WAREHOUSE] + list(ASSETS.values()))
url = {k: f"{assets_root}{v}" for k, v in ASSETS.items()}

stage = open_target_stage(scene_path)
stage.GetRootLayer().documentation = "Generated by tools/isaac/build_factory.py - re-run the script instead of editing by hand."
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())
define_physics_scene(stage)

# --- Hall: reference + deactivate storage clutter ------------------------------------------------
warehouse_url = f"{assets_root}{WAREHOUSE}"
env = UsdGeom.Xform.Define(stage, "/World/Env").GetPrim()
env.GetReferences().AddReference(warehouse_url)
# List the warehouse top level in a SEPARATE in-memory stage to get exact prim names.
wh = Usd.Stage.Open(warehouse_url, Usd.Stage.LoadAll)
wh_root = wh.GetDefaultPrim()
deactivated = []
for child in wh_root.GetChildren():
    if any(child.GetName().startswith(pref) for pref in DEACTIVATE_PREFIXES):
        stage.OverridePrim(f"/World/Env/{child.GetName()}").SetActive(False)
        deactivated.append(child.GetName())
kept = {}  # kept top-level prims grouped by name without the numeric suffix, e.g. "SM_WallA": 34
for c in wh_root.GetChildren():
    if c.GetName() not in deactivated:
        key = re.sub(r"(_?\d+[A-Za-z]?\d*)+$", "", c.GetName())
        kept[key] = kept.get(key, 0) + 1
del wh

# Same ground friction as build_scene.py (policy training config), bound for the "physics" purpose on the
# warehouse floor colliders. Both the floor meshes and the hidden infinite CollisionPlane sit at z = 0.
looks = "/World/Looks"
ground_mat = UsdShade.Material.Define(stage, f"{looks}/GroundPhysics")
pm = UsdPhysics.MaterialAPI.Apply(ground_mat.GetPrim())
pm.CreateStaticFrictionAttr().Set(1.0)
pm.CreateDynamicFrictionAttr().Set(1.0)
pm.CreateRestitutionAttr().Set(0.0)
floor_colliders = []
for child in env.GetChildren():
    if child.GetName().startswith("SM_floor") or child.GetName() == "GroundPlane":
        for p in Usd.PrimRange(child):
            if p.HasAPI(UsdPhysics.CollisionAPI) and not p.IsInstanceProxy():
                UsdShade.MaterialBindingAPI.Apply(p).Bind(ground_mat, UsdShade.Tokens.weakerThanDescendants, "physics")
                floor_colliders.append(p.GetPath().pathString)

# --- Shared materials for procedural parts ------------------------------------------------------
m_paint = preview_material(stage, f"{looks}/PaintedSteelBlue", (0.20, 0.27, 0.36), metallic=0.3, roughness=0.55)
m_steel = preview_material(stage, f"{looks}/Steel", (0.55, 0.56, 0.58), metallic=0.9, roughness=0.35)
m_yellow = preview_material(stage, f"{looks}/SafetyYellow", (0.85, 0.62, 0.06), metallic=0.2, roughness=0.5)

props_report = []
LA, LB, LC = "/World/Props/Line_A_SteamHeader", "/World/Props/Line_B_TankFarm", "/World/Props/Line_C_Utility"
for grp in (LA, LB, LC):
    UsdGeom.Xform.Define(stage, grp)

# --- Line A: steam header ------------------------------------------------------------------------
# Office pipes run along their local +Y; rotate -90 deg about Z so they run along world +X.
for i, x0 in enumerate((-6.0, -3.0, 0.0)):
    props_report.append(add_asset(stage, f"{LA}/Header_{i + 1:02d}", url["pipe_big_3m"], (x0, HEADER_Y, HEADER_Z), -90.0))
    props_report.append(add_asset(stage, f"{LA}/SmallPipe_{i + 1:02d}", url["pipe_small_3m"], (x0, SMALL_Y, SMALL_Z), -90.0))
props_report.append(add_asset(stage, f"{LA}/Valve", url["valve_big"], (-1.5, HEADER_Y, HEADER_Z), -90.0))
for i, x in enumerate((-6.0, -3.0, 0.0, 3.0)):  # flanges at joints, blind flanges at the ends
    add_cylinder(stage, f"{LA}/Flange_Header_{i + 1:02d}", (x, HEADER_Y, HEADER_Z), "X", HEADER_R + 0.04, 0.03, m_steel)
    add_cylinder(stage, f"{LA}/Flange_Small_{i + 1:02d}", (x, SMALL_Y, SMALL_Z), "X", SMALL_R + 0.03, 0.025, m_steel)
post_h1 = HEADER_Z - HEADER_R   # header post top touches the pipe bottom
post_h2 = SMALL_Z - SMALL_R
for i, x in enumerate((-5.6, -3.9, -2.2, -0.4, 1.5, 2.75)):
    s = f"{LA}/Support_{i + 1:02d}"
    UsdGeom.Xform.Define(stage, s)
    add_box(stage, f"{s}/BasePlate", (x, (HEADER_Y + SMALL_Y) / 2, 0.006), (0.22, 0.75, 0.012), m_paint)
    add_box(stage, f"{s}/PostHeader", (x, HEADER_Y, post_h1 / 2), (0.08, 0.08, post_h1), m_paint)
    add_box(stage, f"{s}/PostSmall", (x, SMALL_Y, post_h2 / 2), (0.06, 0.06, post_h2), m_paint)
    add_box(stage, f"{s}/Brace", (x, (HEADER_Y + SMALL_Y) / 2, 0.18), (0.04, abs(SMALL_Y - HEADER_Y), 0.04), m_paint)

# --- Line B: tank farm ---------------------------------------------------------------------------
props_report.append(add_asset(stage, f"{LB}/IBC_Stainless", url["ibc_steel"], (*IBC_STEEL_XY, 0.0), 0.0, "convexHull"))
props_report.append(add_asset(stage, f"{LB}/IBC_Caged", url["ibc_cage"], (-2.2, 3.1, 0.0), 0.0, "convexHull"))
props_report.append(add_asset(stage, f"{LB}/Drum_01", url["drum_a01"], (0.0, 3.0, 0.0), 0.0, "convexHull"))
props_report.append(add_asset(stage, f"{LB}/Drum_02", url["drum_a01"], (0.75, 3.05, 0.0), 40.0, "convexHull"))
props_report.append(add_asset(stage, f"{LB}/Drum_03", url["drum_a02"], (0.4, 3.7, 0.0), 15.0, "convexHull"))
stand = f"{LB}/GaugeStand"
UsdGeom.Xform.Define(stage, stand)
stand_top = STAND_GAUGE_Z - 0.050 - 0.03  # minus gauge radius (D 100 mm) and stem length
add_box(stage, f"{stand}/BasePlate", (*STAND_XY, 0.006), (0.25, 0.25, 0.012), m_yellow)
add_cylinder(stage, f"{stand}/Post", (*STAND_XY, (0.012 + stand_top) / 2), "Z", 0.025, stand_top - 0.012, m_yellow)

# --- Line C: utility corner ----------------------------------------------------------------------
props_report.append(add_asset(stage, f"{LC}/ElectricalCabinet", url["fuse_box"], (2.6, 3.35, 0.0), -90.0))
props_report.append(add_asset(stage, f"{LC}/RailGuard_Cabinet", url["rail_guard"], (2.63, 2.6, 0.0), 0.0, "none"))
props_report.append(add_asset(stage, f"{LC}/RailGuard_SW", url["rail_guard"], (-6.9, -3.0, 0.0), 90.0, "none"))
props_report.append(add_asset(stage, f"{LC}/RailGuard_NW", url["rail_guard"], (-6.9, 3.0, 0.0), 90.0, "none"))
props_report.append(add_asset(stage, f"{LC}/PipeBundle", url["pipe_bundle"], (-1.5, 5.6, 0.0), 90.0, "none"))

# --- Gauges --------------------------------------------------------------------------------------
os.makedirs(texture_dir, exist_ok=True)
UsdGeom.Xform.Define(stage, "/World/Gauges")
gauge_specs = _gauges()
for g in gauge_specs:
    png = os.path.join(texture_dir, f"{g['id'].lower()}_{g['label'].replace('-', '').lower()}.png").replace("\\", "/")
    make_gauge.render_dial_png(png, g["vmin"], g["vmax"], g["unit"], g["label"], g["major"], g["minor"], red_from=g["red_from"])
    g["png"] = png
    make_gauge.add_gauge(
        stage, f"/World/Gauges/{g['id']}", dial_png=png, value=g["value"], vmin=g["vmin"], vmax=g["vmax"],
        diameter_m=g["d"], translate=g["center"], rotate_xyz_deg=(0.0, 0.0, g["rot"]), unit=g["unit"],
        label=g["label"], mount=g["mount"], stem_length_m=g["stem"],
    )

# ------------------------------------------------------------------------------------------------
# Meta JSON + free-space checks (world bounding boxes of everything we added)
# ------------------------------------------------------------------------------------------------
bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])


def world_box(prim):
    r = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return [list(r.GetMin()), list(r.GetMax())]


obstacles = []  # (path, min, max) of every top-level item under /World/Props/<line> and every gauge
for grp in (LA, LB, LC):
    for item in stage.GetPrimAtPath(grp).GetChildren():
        mn, mx = world_box(item)
        obstacles.append((item.GetPath().pathString, mn, mx))
for g in gauge_specs:
    mn, mx = world_box(stage.GetPrimAtPath(f"/World/Gauges/{g['id']}"))
    obstacles.append((f"/World/Gauges/{g['id']}", mn, mx))

# Hall interior = union of wall boxes, shrunk by the wall/column thickness.
walls = None
for child in env.GetChildren():
    if child.GetName().startswith("SM_WallA"):
        r = bbox_cache.ComputeWorldBound(child).ComputeAlignedRange()
        walls = r if walls is None else Gf.Range3d.GetUnion(walls, r)
WALL_MARGIN = 0.35
hall = {"x": [walls.GetMin()[0] + WALL_MARGIN, walls.GetMax()[0] - WALL_MARGIN],
        "y": [walls.GetMin()[1] + WALL_MARGIN, walls.GetMax()[1] - WALL_MARGIN]}


def clearance(x, y):
    """Distance to the nearest obstacle footprint (only things below 1 m matter for the robot) and wall."""
    best, who = float("inf"), None
    for path, mn, mx in obstacles:
        if mn[2] > 1.0:
            continue
        d = rect_distance(x, y, mn, mx)
        if d < best:
            best, who = d, path
    wall = min(x - hall["x"][0], hall["x"][1] - x, y - hall["y"][0], hall["y"][1] - y)
    return best, who, wall


problems = []
sx, sy, _ = ROBOT_SPAWN["position"]
spawn_clear, spawn_who, spawn_wall = clearance(sx, sy)
if spawn_clear < SPAWN_MIN_CLEARANCE_M or spawn_wall < SPAWN_MIN_CLEARANCE_M:
    problems.append(f"spawn clearance {spawn_clear:.2f} m ({spawn_who}), wall {spawn_wall:.2f} m")

# Aisle: nothing may cross the centerline band; report the narrowest free width along the aisle.
south_edge, north_edge = -float("inf"), float("inf")
for path, mn, mx in obstacles:
    if mx[0] < AISLE["x_range"][0] or mn[0] > AISLE["x_range"][1] or mn[2] > 1.0:
        continue
    if mn[1] < 0.0 < mx[1]:
        problems.append(f"{path} crosses the aisle centerline")
    elif mx[1] <= 0.0:
        south_edge = max(south_edge, mx[1])
    else:
        north_edge = min(north_edge, mn[1])
aisle_width = north_edge - south_edge
if aisle_width < 1.5:
    problems.append(f"aisle clear width {aisle_width:.2f} m < 1.5 m")

gauges_meta = []
for g in gauge_specs:
    gt = make_gauge.gauge_ground_truth(stage, f"/World/Gauges/{g['id']}")
    n = gt["face_normal_world"]
    nh = math.hypot(n[0], n[1])
    nx, ny = n[0] / nh, n[1] / nh
    c = gt["center_world"]
    wx, wy = c[0] + WAYPOINT_STANDOFF_M * nx, c[1] + WAYPOINT_STANDOFF_M * ny
    yaw = math.degrees(math.atan2(-ny, -nx))  # robot looks back at the gauge
    wclear, wwho, wwall = clearance(wx, wy)
    if wclear < ROBOT_HALF_LENGTH_M or wwall < ROBOT_HALF_LENGTH_M:
        problems.append(f"{g['id']} waypoint clearance {wclear:.2f} m ({wwho}), wall {wwall:.2f} m")
    gt.update({
        "id": g["id"],
        "mounted_on": g["on"],
        "mount": g["mount"],
        "waypoint": {"x": round(wx, 3), "y": round(wy, 3), "yaw_deg": round(yaw, 1),
                     "standoff_m": WAYPOINT_STANDOFF_M, "clearance_m": round(wclear, 3)},
    })
    gauges_meta.append(gt)

meta = {
    "scene": os.path.basename(scene_path),
    "generated_by": "tools/isaac/build_factory.py",
    "units": "meters, Z up, floor at z = 0, yaw in degrees counter-clockwise from +X",
    "assets_root": assets_root,
    "environment": {"reference": warehouse_url, "prim": "/World/Env", "deactivated": deactivated, "kept": kept},
    "hall_interior_xy": {k: [round(v, 3) for v in vals] for k, vals in hall.items()},
    "aisle": dict(AISLE, clear_width_m=round(aisle_width, 3), south_edge_y=round(south_edge, 3), north_edge_y=round(north_edge, 3)),
    "robot_spawn": dict(ROBOT_SPAWN, clearance_m=round(spawn_clear, 3),
                        note="On the floor in the aisle facing +X along it. start_go2.py drops the robot from z=0.5; "
                             "its current default (0, 0) is also free aisle space."),
    "camera_assumption": {"front_camera_offset_from_base_m": 0.327, "vertical_fov_deg": 45.2,
                          "camera_to_dial_at_waypoint_m": round(WAYPOINT_STANDOFF_M - 0.327, 2)},
    "gauge_convention": "dial faces gauge-local +X, 12 o'clock = local +Z; needle_angle_deg is clockwise from "
                        "12 o'clock seen from the front; vmin at -sweep/2, vmax at +sweep/2",
    "inspection_route": [g["id"] for g in sorted(gauges_meta, key=lambda m: (m["waypoint"]["x"], m["waypoint"]["y"]))],
    "gauges": gauges_meta,
    "props": [
        {"prim_path": p, "bbox_min": [round(v, 3) for v in mn], "bbox_max": [round(v, 3) for v in mx]}
        for p, mn, mx in obstacles if not p.startswith("/World/Gauges")
    ],
    "physics_overrides": props_report,
    "floor_physics_material_bound_to": floor_colliders,
}

if problems:
    raise RuntimeError("layout check failed (scene NOT saved):\n  " + "\n  ".join(problems))

stage.GetRootLayer().Save()
with open(meta_path, "w", encoding="utf-8") as f:
    json.dump(meta, f, indent=2, ensure_ascii=False)

print(f"saved {scene_path}")
print(f"saved {meta_path}")
print(f"deactivated {len(deactivated)} warehouse prims; floor colliders with friction 1.0: {len(floor_colliders)}")
print(f"aisle clear width {aisle_width:.2f} m (y {south_edge:.2f} .. {north_edge:.2f}); spawn clearance {spawn_clear:.2f} m")
for p in props_report:
    print(f"  {p['prim_path']}: variant={p['physics_variant']} colliders={p['colliders']} approx={p['approximations']} "
          f"rb_disabled={p['rigid_bodies_disabled']} deinstanced={p['deinstanced']}")
for m in gauges_meta:
    w = m["waypoint"]
    print(f"  {m['id']} {m['label']}: {m['value']} {m['unit']} [{m['vmin']}..{m['vmax']}] center={m['center_world']} "
          f"normal={m['face_normal_world']} waypoint=({w['x']}, {w['y']}, {w['yaw_deg']} deg) clear={w['clearance_m']}")
