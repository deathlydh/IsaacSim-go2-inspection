"""Print prims under a root that carry collision or physics-material bindings."""

import omni.usd
from pxr import Usd, UsdPhysics

if "root" not in dir():
    root = "/World/ground"

stage = omni.usd.get_context().get_stage()
for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
    if prim.HasAPI(UsdPhysics.CollisionAPI) or prim.GetTypeName() in ("Plane", "Mesh"):
        print(prim.GetPath(), prim.GetTypeName(), "collision" if prim.HasAPI(UsdPhysics.CollisionAPI) else "")
