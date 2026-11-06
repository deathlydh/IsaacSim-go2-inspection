"""Read-only render-geometry diagnosis for virtual dataset camera occlusion."""
import omni.usd
from pxr import Usd,UsdGeom
stage=omni.usd.get_context().get_stage()
for p in Usd.PrimRange(stage.GetPrimAtPath('/World/Robot')):
    path=str(p.GetPath())
    if p.IsInstance() or 'visual' in path.lower() or p.IsA(UsdGeom.Gprim):
        print(path,p.GetTypeName(),'instance',p.IsInstance(),'visibility',UsdGeom.Imageable(p).ComputeVisibility() if p.IsA(UsdGeom.Imageable) else None)
