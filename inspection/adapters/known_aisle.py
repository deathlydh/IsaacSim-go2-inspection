"""Known-static aisle navigation adapter, with configurable origin/orientation.

Not a general planner: refuses unchecked segments; no obstacle avoidance.
"""
import math
from inspection.actions import approach_pose
from inspection.geometry import Clearance


def transform(x,y,origin,angle,inverse=False):
    c,s = math.cos(angle),math.sin(angle)
    if inverse:
        x,y = x-origin[0],y-origin[1]
        return [c*x+s*y,-s*x+c*y]
    return [origin[0]+c*x-s*y,origin[1]+s*x+c*y]


def plan_route(start, gauges, aisle):
    # Legacy entry point, retained for saved tools/tests.
    adapter = KnownAisle({'origin_xy':[0,aisle['centerline_y']], 'yaw_deg':0,
                          'along_range':[-1e6,1e6],'cross_range':[-1e6,1e6]},[],0)
    from inspection.actions import Pose
    converted = [{**g,'inspection_pose':g['waypoint']} for g in gauges]
    return adapter.plan(Pose(start[0],start[1],.3,start[2],0),converted)


class KnownAisle:
    def __init__(self, config, obstacles, radius, excluded=()):
        self.config = config
        self.origin = config['origin_xy']
        self.angle = math.radians(config['yaw_deg'])
        self.obstacles,self.excluded = obstacles,list(excluded)
        self.clearance = Clearance(obstacles,radius,[-math.inf,math.inf,-math.inf,math.inf])

    def blockers(self,a,b):
        bounds = self.config['along_range']+self.config['cross_range']
        for p in (a,b):
            x,y = transform(p[0],p[1],self.origin,self.angle,True)
            if not all(math.isfinite(t) for t in (x,y)) or not bounds[0]<=x<=bounds[1] or not bounds[2]<=y<=bounds[3]:
                return ['outside_known_aisle']
        return self.clearance.blockers(a,b)

    def plan(self,start,gauges):
        x,y = transform(start.x,start.y,self.origin,self.angle,True)
        points=[]
        def add(px,py,heading,phase,gid=None,keep=False):
            wx,wy=transform(px,py,self.origin,self.angle)
            points.append({'target':[wx,wy,heading+self.angle], 'phase':phase,'gauge_id':gid,'keep_heading':keep})
        if abs(y)>.05:
            add(x,0,start.yaw-self.angle,'join_aisle',keep=True)
        for index,g in enumerate(gauges):
            wp=g['inspection_pose']
            tx,ty=transform(wp['x'],wp['y'],self.origin,self.angle,True)
            yaw=math.radians(wp['yaw_deg'])-self.angle
            add(tx,0,0 if tx>=x else math.pi,'transit',g['id'])
            add(tx,ty,yaw,'approach',g['id'])
            if index<len(gauges)-1:
                add(tx,0,yaw,'return_aisle',g['id'],True)
            x,y=tx,ty
        return points

    async def navigate_to(self,io,point,cfg,event=None):
        return await approach_pose(io,point['target'],cfg,event=event,keep_heading=point['keep_heading'])

    def cancel(self,io): io.cancel()
