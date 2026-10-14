"""List folders/files in the Isaac Sim asset library (cloud) under given sub-paths.

Injected args: paths (list of sub-paths relative to the assets root), depth (int, default 1)
"""

import omni.client
from isaacsim.storage.native import get_assets_root_path

if "paths" not in dir():
    paths = ["/Isaac/Environments"]
if "depth" not in dir():
    depth = 1

root = get_assets_root_path()
print("assets root:", root)


async def walk(url, level, indent=""):
    result, entries = await omni.client.list_async(url)
    if result != omni.client.Result.OK:
        print(indent + f"[{result}] {url}")
        return
    for e in sorted(entries, key=lambda e: e.relative_path):
        is_dir = bool(e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN)
        size = "" if is_dir else f"  ({e.size / 1e6:.1f} MB)"
        print(f"{indent}{e.relative_path}{'/' if is_dir else ''}{size}")
        if is_dir and level < depth:
            await walk(f"{url}/{e.relative_path}", level + 1, indent + "    ")


for p in paths:
    print(f"=== {p}")
    await walk(f"{root}{p}", 1)
