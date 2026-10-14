import omni.usd, omni.timeline, omni.kit.app
from pxr import PhysxSchema

# args: gpu (bool)
stage = omni.usd.get_context().get_stage()
tl = omni.timeline.get_timeline_interface()
app = omni.kit.app.get_app()
ps = PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath("/PhysicsScene"))
tl.stop()
for _ in range(5):
    await app.next_update_async()
ps.GetEnableGPUDynamicsAttr().Set(bool(gpu))
ps.GetBroadphaseTypeAttr().Set("GPU" if gpu else "MBP")
for _ in range(5):
    await app.next_update_async()
print("gpu dynamics:", ps.GetEnableGPUDynamicsAttr().Get(), "| broadphase:", ps.GetBroadphaseTypeAttr().Get(), "| playing:", tl.is_playing())
