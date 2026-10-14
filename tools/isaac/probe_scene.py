import omni.usd, omni.timeline, carb
from pxr import UsdPhysics, PhysxSchema

stage = omni.usd.get_context().get_stage()
tl = omni.timeline.get_timeline_interface()
print("stage:", stage.GetRootLayer().identifier if stage else None)
print("playing:", tl.is_playing(), "| time codes/s:", tl.get_time_codes_per_seconds())
for p in stage.Traverse():
    if p.IsA(UsdPhysics.Scene):
        ps = PhysxSchema.PhysxSceneAPI(p)
        print("physics scene:", p.GetPath(),
              "| steps/s:", ps.GetTimeStepsPerSecondAttr().Get(),
              "| solver:", ps.GetSolverTypeAttr().Get(),
              "| gpu dynamics:", ps.GetEnableGPUDynamicsAttr().Get(),
              "| broadphase:", ps.GetBroadphaseTypeAttr().Get())
s = carb.settings.get_settings()
for k in ["/rtx/rendermode", "/app/runLoops/main/rateLimitEnabled", "/app/runLoops/main/rateLimitFrequency",
          "/persistent/physics/numThreads", "/physics/numThreads", "/app/player/useFixedTimeStepping"]:
    print(k, "=", s.get(k))
print("articulations:", [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)])
print("prims total:", sum(1 for _ in stage.Traverse()))
