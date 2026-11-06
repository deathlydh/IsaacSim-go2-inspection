"""Reproducible collection conditions, declared before evaluation. No CV imports."""
import random


def make_plan(gauge_ids,per_gauge=40,seed=1701):
    if per_gauge<2 or per_gauge%2: raise ValueError('per_gauge must be even and >=2')
    plan=[]
    for gi,gid in enumerate(gauge_ids):
        for j in range(per_gauge):
            split='dev' if j<per_gauge//2 else 'test'
            local=j%(per_gauge//2)
            sample_seed=seed+gi*10000+local+(1000000 if split=='test' else 0)
            rng=random.Random(sample_seed)
            fraction=(0.0 if local==0 else 1.0 if local==1 else rng.uniform(.02,.98))
            # Disjoint distance conditions as well as seeds; not random adjacent frames.
            distance=rng.uniform(.8,1.15) if split=='dev' else rng.uniform(1.25,1.6)
            plan.append({'sample_id':f'{gid}_{j:04d}','gauge_id':gid,'seed':sample_seed,'split':split,
                         'value_fraction':fraction,'distance_m':distance,
                         'yaw_offset_deg':rng.uniform(-10,10) if split=='dev' else rng.uniform(-15,15),
                         'light_scale':(.8 if local%2==0 else 1.15),
                         'hidden':per_gauge>=20 and local==per_gauge//2-1})
    return plan
