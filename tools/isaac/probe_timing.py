"""Print timing-related settings that decide physics steps per rendered frame."""

import carb
import omni.timeline
from isaacsim.core.simulation_manager import SimulationManager

s = carb.settings.get_settings()
tl = omni.timeline.get_timeline_interface()
print("timeline time codes/s:", tl.get_time_codes_per_seconds(), "| target framerate:", tl.get_target_framerate()
      if hasattr(tl, "get_target_framerate") else "n/a")
print("physics dt:", SimulationManager.get_physics_dt())
for k in ["/app/runLoops/main/rateLimitEnabled", "/app/runLoops/main/rateLimitFrequency",
          "/app/runLoops/main/manualModeEnabled", "/app/player/useFixedTimeStepping",
          "/app/player/useFastMode", "/persistent/simulation/minFrameRate", "/physics/minFrameRate",
          "/persistent/physics/minFrameRate", "/app/runLoops/main/manualStepSize"]:
    print(k, "=", s.get(k))
