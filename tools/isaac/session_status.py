"""Print the live Go2 session state (policy, ready/failed, command, timeline, pose)."""

import sys

import numpy as np
import omni.timeline

s = getattr(sys, "_go2_inspection_session", None)
tl = omni.timeline.get_timeline_interface()
print("timeline playing:", tl.is_playing(), "| time:", round(tl.get_current_time(), 2))
if s is None:
    print("no session")
else:
    print("policy:", s.policy, "| ready:", s.ready, "| failed:", s.failed, "| command:", s.command.tolist(),
          "| heading target:", s._heading_target, "| callbacks:", len(s._ids), "| runner:", s.runner is not None)
    if s.runner is not None and s.runner.articulation is not None:
        pos, quat = s.runner.articulation.get_world_poses()
        print("pose:", np.asarray(pos.numpy() if hasattr(pos, "numpy") else pos).round(3).tolist(),
              "| tensors valid:", s.runner.articulation.is_physics_tensor_entity_valid())
