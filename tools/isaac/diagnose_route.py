"""Read-only current pose and clearance diagnostic; does not set commands."""
import sys
import json
from dataclasses import asdict
from pathlib import Path
sys.path.insert(0,"D:/go2-inspection/tools/isaac")
import inspection_actions
import route_planning
import omni.usd
s = sys._go2_inspection_session
io = inspection_actions.KitIO(s,s.control)
p = io.pose()
obstacles, excluded = route_planning.stage_obstacles(omni.usd.get_context().get_stage())
output = {"pose":asdict(p),"command":s.command.tolist(),"checks":[]}
for radius in (0.5,0.55,0.6,0.65,0.7):
    c = route_planning.Clearance(obstacles,radius,[-9,8.5,-1.607,1.607])
    blockers = c.blockers([p.x,p.y],[p.x,0])
    output["checks"].append({"radius":radius,"blockers":blockers,
                             "objects":[o for o in obstacles if o["path"] in blockers]})
print(json.dumps(output))
