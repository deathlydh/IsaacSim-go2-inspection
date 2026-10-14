"""Build the base inspection scene and save it to disk.

Environment only (ground, lights, physics settings). The robot and its policy are added at
runtime by start_go2.py, so the saved file never holds a stale robot.

Injected args (isaacsim_send.py --arg):
    scene_path: output .usd path (default D:/go2-inspection/scenes/inspection.usd)
"""

import omni.kit.app
import omni.usd
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

if "scene_path" not in dir():
    scene_path = "D:/go2-inspection/scenes/inspection.usd"

import omni.timeline

ctx = omni.usd.get_context()
# Stop explicitly and let the Stop handlers finish while the old stage is still alive:
# Isaac Sim 6.1 crashes (virtual_gantry -> debug_draw) if Stop fires during stage teardown.
omni.timeline.get_timeline_interface().stop()
for _ in range(10):
    await omni.kit.app.get_app().next_update_async()
await ctx.new_stage_async()
await omni.kit.app.get_app().next_update_async()
stage = ctx.get_stage()

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

# Ground + lights from the same environment the Go2 policy example uses.
assets_root = get_assets_root_path()
ground = stage.DefinePrim("/World/ground", "Xform")
ground.GetReferences().AddReference(f"{assets_root}/Isaac/Environments/Grid/default_environment.usd")

# Ground friction matching the policy's training config.
material = UsdShade.Material.Define(stage, "/World/Looks/GroundPhysics")
phys_mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
phys_mat.CreateStaticFrictionAttr().Set(1.0)
phys_mat.CreateDynamicFrictionAttr().Set(1.0)
phys_mat.CreateRestitutionAttr().Set(0.0)
# The referenced environment streams from the cloud, so author the binding as an "over" on top of
# the reference: it composes onto the collision plane whenever the reference resolves.
collision_plane = stage.OverridePrim("/World/ground/GroundPlane/CollisionPlane")
UsdShade.MaterialBindingAPI.Apply(collision_plane).Bind(
    material, UsdShade.Tokens.weakerThanDescendants, "physics"
)

# Physics: 200 Hz (policy training rate), CPU solver - faster than GPU for a single robot.
scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)
physx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
physx.CreateTimeStepsPerSecondAttr().Set(200)
physx.CreateSolverTypeAttr().Set("TGS")
physx.CreateEnableGPUDynamicsAttr().Set(False)
physx.CreateBroadphaseTypeAttr().Set("MBP")

ok = await ctx.save_as_stage_async(scene_path)
print("saved:", ok, scene_path)
# save_as re-opens the stage, so re-fetch the prim from the saved stage before checking.
plane = ctx.get_stage().GetPrimAtPath("/World/ground/GroundPlane/CollisionPlane")
binding = UsdShade.MaterialBindingAPI(plane).GetDirectBinding("physics").GetMaterialPath() if plane else None
print("ground physics material:", binding)
