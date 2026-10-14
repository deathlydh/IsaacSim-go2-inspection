"""Inspect a saved scene file directly (no stage switch): physics settings and ground friction."""

from pxr import PhysxSchema, Usd, UsdShade

if "scene_path" not in dir():
    scene_path = "D:/go2-inspection/scenes/inspection.usd"

stage = Usd.Stage.Open(scene_path)
print("default prim:", stage.GetDefaultPrim().GetPath())
ps = PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath("/PhysicsScene"))
print("steps/s:", ps.GetTimeStepsPerSecondAttr().Get(), "| gpu dynamics:", ps.GetEnableGPUDynamicsAttr().Get(),
      "| broadphase:", ps.GetBroadphaseTypeAttr().Get(), "| solver:", ps.GetSolverTypeAttr().Get())
plane = stage.GetPrimAtPath("/World/ground/GroundPlane/CollisionPlane")
print("collision plane composed:", bool(plane))
if plane:
    print("physics material:", UsdShade.MaterialBindingAPI(plane).GetDirectBinding("physics").GetMaterialPath())
print("robot in file (should be False):", stage.GetPrimAtPath("/World/Robot").IsValid())
