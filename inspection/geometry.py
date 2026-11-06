"""Conservative swept-envelope checks for a known static factory scene."""
import math


def intersects_box(a, b, box, radius):
    """Analytical slab test against an expanded AABB, including both endpoints."""
    low, high = 0.0, 1.0
    for i in (0, 1):
        mn, mx = box[0][i]-radius, box[1][i]+radius
        delta = b[i]-a[i]
        if abs(delta) < 1e-12:
            if not mn <= a[i] <= mx:
                return False
        else:
            p, q = (mn-a[i])/delta, (mx-a[i])/delta
            low, high = max(low,min(p,q)), min(high,max(p,q))
            if low > high:
                return False
    return True


class Clearance:
    def __init__(self, obstacles, radius, bounds):
        self.obstacles, self.radius, self.bounds = obstacles, radius, bounds

    def blockers(self, a, b):
        if not all(math.isfinite(v) for v in (*a[:2], *b[:2])):
            return ["nonfinite_position"]
        xmin,xmax,ymin,ymax = self.bounds
        if any(not (xmin <= p[0] <= xmax and ymin <= p[1] <= ymax) for p in (a,b)):
            return ["outside_known_aisle"]
        return [o["path"] for o in self.obstacles if intersects_box(a,b,o["box"],self.radius)]

