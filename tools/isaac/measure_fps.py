import asyncio, time, statistics
import omni.kit.app, omni.timeline, omni.physx

app = omni.kit.app.get_app()
tl = omni.timeline.get_timeline_interface()
physx = omni.physx.get_physx_interface()

phys_ms = []
_t = {}

def on_pre(dt):
    _t["s"] = time.perf_counter()

def on_post(dt):
    if "s" in _t:
        phys_ms.append((time.perf_counter() - _t.pop("s")) * 1000)

async def run(warmup=2.0, seconds=6.0):
    was_playing = tl.is_playing()
    if not was_playing:
        tl.play()
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < warmup:
        await app.next_update_async()
    # physics step timing via simulation event stream
    sim = omni.physx.get_physx_simulation_interface()
    sub = physx.subscribe_physics_step_events(on_post)
    pre = physx.subscribe_physics_on_step_events(on_pre, pre_step=True, order=-10000) if hasattr(physx, "subscribe_physics_on_step_events") else None
    frames = []
    last = time.perf_counter(); t0 = last
    while time.perf_counter() - t0 < seconds:
        await app.next_update_async()
        now = time.perf_counter(); frames.append((now - last) * 1000); last = now
    sub = None; pre = None
    if not was_playing:
        tl.pause()
    fps = len(frames) / seconds
    print(f"frames: {len(frames)} in {seconds:.0f}s -> avg FPS {fps:.1f}")
    print(f"frame ms: median {statistics.median(frames):.1f}, p90 {sorted(frames)[int(len(frames)*0.9)]:.1f}, max {max(frames):.1f}")
    if phys_ms:
        steps_per_frame = len(phys_ms) / max(1, len(frames))
        print(f"physics steps: {len(phys_ms)} ({steps_per_frame:.1f}/frame)")
    print("physics step-callback timing (pre->post) samples:", len(phys_ms))
    if phys_ms:
        print(f"  per-step ms median {statistics.median(phys_ms):.2f} -> per-frame ~{statistics.median(phys_ms)*steps_per_frame:.1f} ms")

await run()
