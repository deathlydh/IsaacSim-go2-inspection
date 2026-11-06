"""Load static site/run configuration. Truth is never a site/CV setting."""
import hashlib
import json
import math
from pathlib import Path


def load_site(path):
    path = Path(path).resolve()
    raw = path.read_bytes()
    site = json.loads(raw)
    if site.get('schema_version') != 1:
        raise ValueError('unsupported_site_schema')
    ids = [g['id'] for g in site['gauges']]
    if not ids or len(set(ids)) != len(ids):
        raise ValueError('duplicate_or_missing_gauge_ids')
    if not site['route'] or any(i not in ids for i in site['route']):
        raise ValueError('invalid_route')
    stations = {s['id']:s for s in site['stations']}
    if len(stations) != len(site['stations']):
        raise ValueError('duplicate_station_ids')
    for g in site['gauges']:
        if g['station_id'] not in stations or g['id'] not in stations[g['station_id']]['gauge_ids']:
            raise ValueError('invalid_station_membership')
        if any(k in g or k in g['calibration'] for k in ('value','truth','needle_angle_deg','center_world')):
            raise ValueError('truth_forbidden_in_static_site')
        p = g['inspection_pose']
        if not all(math.isfinite(float(p[k])) for k in ('x','y','yaw_deg')):
            raise ValueError('invalid_inspection_pose')
    if site['map']['units'] != 'm' or not site['map'].get('z_up'):
        raise ValueError('unsupported_coordinate_convention')
    transform=site['map'].get('world_from_map',{'translation_m':[0,0,0],'yaw_deg':0})
    if any(abs(float(v))>1e-9 for v in transform['translation_m']) or abs(float(transform['yaw_deg']))>1e-9:
        raise ValueError('nonidentity_map_transform_requires_localization_adapter')
    if site['navigation']['adapter'] != 'known_aisle':
        raise ValueError('unknown_navigation_adapter')
    site['scene'] = str((path.parent/site['scene']).resolve())
    site['source_path'] = str(path)
    site['sha256'] = hashlib.sha256(raw).hexdigest()
    return site


def load_config(path, site_override=None):
    path = Path(path).resolve()
    config = json.loads(path.read_text(encoding='utf-8'))
    config['reports_root'] = str((path.parent/config['reports_root']).resolve())
    site_path = Path(site_override).resolve() if site_override else (path.parent/config['site_path']).resolve()
    site = load_site(site_path)
    config['site_path'] = str(site_path)
    return config, site


def verify_scene(site, loaded_path):
    if site.get('offline_only'):
        raise ValueError('offline_only_site')
    if Path(loaded_path).resolve() != Path(site['scene']).resolve():
        raise ValueError('loaded_scene_does_not_match_site')
