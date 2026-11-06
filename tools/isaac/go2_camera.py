"""Front RGB camera on the Unitree Go2 (+ an optional second viewport window that shows it).

WHAT this module does
---------------------
``FrontCamera.create()`` adds the Go2's standard front camera to the robot that is already on the stage:

    /World/Robot/base                    robot body (a physics rigid body that moves with the robot)
      front_camera   [Xform]             mount point, translate (0.32715, 0, 0.04297) m in body axes
        rgb          [Camera]            the lens; RTX sensor (OmniSensorAPI) rendering at 10 Hz

and (optionally) a floating 640x360 viewport window titled "Go2 Front Camera" that shows what the robot sees.
Pixels are read with ``get_rgb()`` (numpy HxWx3 uint8), saved with ``save_png()``.

WHY these numbers
-----------------
* Mount offset: official Unitree ``go2_description.urdf`` (unitreerobotics/unitree_ros, BSD-3), the fixed
  joint of the front camera link. It is the front face of the head, in body axes (x forward, y left, z up).
* Lens: real calibration of the Go2 front camera at 1280x720 has fx = 864.4 px, i.e. horizontal field of view
  2*atan(640/864.4) = 73.0 deg, vertical 45.2 deg. (The advertised "120 deg" is not a pinhole FOV.)
* Orientation: a USD camera looks along its own -Z axis with +Y as "image up". The quaternion
  wxyz (0.5, 0.5, -0.5, -0.5) turns it so it looks along the robot's +X (forward) with image-up = robot +Z.
  In Euler angles (the GUI's "Rotate" field, XYZ order) that is (90, 0, -90) degrees.
* 10 Hz: divides the 50 Hz policy/render loop evenly (the camera renders every 5th frame) and is plenty for
  reading gauges while standing still.
* Everything is authored on the stage's SESSION layer, so File > Save never writes the camera into
  scenes/inspection.usd (the camera is re-created by script each run, like the robot itself).

How to do the same by hand in the GUI (Isaac Sim 6.1)
-----------------------------------------------------
1. Stage panel: right-click ``/World/Robot/base`` > Create > Xform, rename it ``front_camera``.
   Property panel > Transform > Translate = (0.32715, 0, 0.04297).
   (Do not put it under ``base/visuals``: that prim is instanceable and cannot take children.)
2. Right-click ``front_camera`` > Create > Camera, rename it ``rgb``.
   Transform > Translate = (0, 0, 0), Rotate = (90, 0, -90).
   Camera properties: Focal Length 24.31, Horizontal Aperture 36, Vertical Aperture 20.25,
   Clipping Range 0.02 / 100. Only the RATIO focal/aperture matters: fx = width * 24.31 / 36.
3. Second window: Window > Viewport > Viewport 2. In that window's camera menu (top-left button, usually
   "Perspective") choose Cameras > rgb; set its resolution to 640x360 in the window's settings menu.
   Alternatively Tools > Sensors > Camera Inspector > Create Viewport.
   Do not drag inside that window: viewport navigation moves the camera it looks through
   (this module sets ``omni:kit:cameraLock`` to prevent that).
4. A GUI-made camera has no ``OmniSensorAPI``, so it renders every frame (no 10 Hz limit), and pixel
   access needs Replicator/Python anyway, so in practice use this module.

Typical use (inside Isaac Sim, e.g. via isaacsim_send.py; the timeline must be playing to get frames)
------------------------------------------------------------------------------------------------------
    import sys; sys.path.insert(0, "D:/go2-inspection/tools/isaac")
    from go2_camera import FrontCamera
    cam = FrontCamera.create()                 # 1280x720 sensor @ 10 Hz + 640x360 window
    rgb = await cam.wait_for_rgb()             # or cam.get_rgb() later; None until the first frame
    cam.save_png("D:/go2-inspection/shots/front.png")
    print(cam.intrinsics())                    # {'fx': 864.4, 'fy': 864.4, 'cx': 640.0, 'cy': 360.0, ...}
    cam.hide_window(); cam.show_window()       # the window costs GPU time every frame; hide it when unused
    rgb = await cam.wait_for_rgb(fresh=True)   # a frame rendered after this call (e.g. once the robot stopped)
    cam.close()                                # window -> sensor/render product -> prims

``create()`` keeps a strong reference to the camera (``FrontCamera.live()`` returns it), so the render
product is not garbage-collected when a script's local variables disappear. ``create()`` on a robot that
already has a camera closes the old one first. Call ``close()`` BEFORE opening/replacing the stage.

Cost measured on this PC (RTX 4060 Ti, Go2 walking scene, interleaved A/B, median frame time):
no camera 30.8 ms, + sensor 720p@10Hz 34.2 ms (+3.4), + 640x360 window 36.6 ms (+2.4 more);
roughly 32 -> 27 -> 26 FPS. The sensor really renders only 10 times per sim-second (counted); most of its
cost is per-frame overhead of having an extra render product at all. VRAM: about +0.35 GB. To get the FPS
back between inspection points, close() the camera and create() it again when needed (do NOT disable the
render product's updates instead: see the note above ``render_count``).

Reloading this module while iterating: Kit caches os.stat() for imports (omni.ext._impl.stat_cache), so
``importlib.reload(go2_camera)`` after editing the file can silently re-run the OLD bytecode. Clear it first:
``from omni.ext._impl import stat_cache; stat_cache._stat_cache.clear(); importlib.reload(go2_camera)``.

API notes (verified against the Isaac Sim 6.1 sources in D:/isaacsim/exts)
--------------------------------------------------------------------------
* ``isaacsim.sensors.experimental.rtx.RtxCamera`` / ``CameraSensor`` replace the deprecated
  ``isaacsim.sensors.camera.Camera``.
* ``CameraSensor(resolution=...)`` takes (HEIGHT, WIDTH); ``ViewportManager.create_viewport_window(
  resolution=...)`` takes (WIDTH, HEIGHT). This module's own arguments are always (WIDTH, HEIGHT).
* ``RtxCamera(orientations=...)`` means WORLD orientation, so the local mounting rotation is set with
  ``set_local_poses()``.
* ``Camera.set_focal_lengths()`` / ``set_apertures()`` multiply the value by 10 before writing the USD
  attribute (legacy convention: argument in cm, USD attribute in mm). We pass mm / 10 so the Property panel
  shows the familiar 24.31 / 36 / 20.25.
* ``CameraSensor`` has no public close(); its render product is released by ``_invalidate_sensor()``
  (what its ``__del__`` calls). We call it explicitly so the release happens now, not "whenever GC runs".
"""

from __future__ import annotations

import asyncio
import math
import os
import re
import struct
import sys
import time
import weakref
import zlib

import carb
import numpy as np
import omni.kit.app
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdRender

# --- Mounting (official Unitree go2_description.urdf, BSD-3) -------------------------------------------
ROBOT_PATH = "/World/Robot"
MOUNT_NAME = "front_camera"  # Xform under <robot>/base
CAMERA_NAME = "rgb"  # Camera under the mount
MOUNT_TRANSLATION = (0.32715, 0.0, 0.04297)  # metres, robot body frame (x fwd, y left, z up)
CAMERA_ORIENTATION_WXYZ = (0.5, 0.5, -0.5, -0.5)  # USD camera (-Z fwd, +Y up) -> robot (+X fwd, +Z up)

# --- Lens (Go2 front camera calibration, 1280x720: fx = 864.4 px) ------------------------------------
CALIBRATION_WIDTH_PX = 1280
CALIBRATION_FX_PX = 864.4
HORIZONTAL_APERTURE_MM = 36.0  # a "35 mm full-frame" sensor width; any value works, only the ratio matters
FOCAL_LENGTH_MM = HORIZONTAL_APERTURE_MM * CALIBRATION_FX_PX / CALIBRATION_WIDTH_PX  # 24.31 mm -> HFOV 73.0 deg
CLIPPING_RANGE_M = (0.02, 100.0)  # near 2 cm: the lens sits a few mm inside the head shell
# isaacsim.core.experimental.objects.Camera multiplies focal length / apertures by this before writing USD.
_WRAPPER_LENS_SCALE = 10.0

# --- Second window --------------------------------------------------------------------------------------
WINDOW_TITLE = "Go2 Front Camera"

# Strong references to live cameras, keyed by mount path. Stored on `sys` (the project convention, cf.
# sys._go2_inspection_session) so it survives importlib.reload() of this module and the end of the
# isaacsim_send function scope; without a reference the CameraSensor would be garbage-collected.
_REGISTRY_ATTR = "_go2_front_cameras"


def _registry() -> dict:
    reg = getattr(sys, _REGISTRY_ATTR, None)
    if reg is None:
        reg = {}
        setattr(sys, _REGISTRY_ATTR, reg)
    return reg


def _write_png(path: str, image: np.ndarray) -> None:
    """Write an HxWx3 uint8 image as PNG. Uses Pillow if present, else a tiny zlib encoder (no dependencies)."""
    try:
        from PIL import Image

        Image.fromarray(image).save(path, format="PNG")
        return
    except ImportError:
        pass
    height, width, _ = image.shape
    raw = b"".join(b"\x00" + image[row].tobytes() for row in range(height))  # filter type 0 per row

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)  # 8-bit RGB
    encoded = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b"")
    if hasattr(path, "write"):
        path.write(encoded)
    else:
        with open(path, "wb") as f:
            f.write(encoded)


class FrontCamera:
    """The Go2 front camera: USD prims + RTX camera sensor (render product + annotators) + optional window.

    Create it with :meth:`create`, not with the constructor. All sizes given to / returned by this class
    are (WIDTH, HEIGHT) unless stated otherwise; numpy images are (HEIGHT, WIDTH, ...).
    """

    def __init__(self, robot_path: str, resolution: tuple[int, int], window_resolution: tuple[int, int],
                 window_title: str, depth: bool):
        self.robot_path = robot_path.rstrip("/")
        self.mount_path = f"{self.robot_path}/base/{MOUNT_NAME}"
        self.camera_path = f"{self.mount_path}/{CAMERA_NAME}"
        self.resolution = (int(resolution[0]), int(resolution[1]))  # (W, H)
        self.window_resolution = (int(window_resolution[0]), int(window_resolution[1]))  # (W, H)
        self.window_title = window_title
        self.depth_enabled = depth
        self.rtx_camera = None  # isaacsim.sensors.experimental.rtx.RtxCamera (USD prim wrapper)
        self.sensor = None  # isaacsim.sensors.experimental.rtx.CameraSensor (render product + annotators)
        self.window = None  # omni.kit.viewport.window.ViewportWindow
        self._fixup_task = None  # asyncio task of _window_fixup()
        self._render_sub = None  # subscription to the sensor's render-completion events
        self._render_count = 0
        self._closed = False

    # ------------------------------------------------------------------ creation
    @classmethod
    def create(
        cls,
        robot_path: str = ROBOT_PATH,
        resolution: tuple[int, int] = (1280, 720),
        tick_rate: float = 10.0,
        window: bool = True,
        window_resolution: tuple[int, int] = (640, 360),
        depth: bool = False,
        window_title: str = WINDOW_TITLE,
    ) -> "FrontCamera":
        """Add the front camera to the robot at ``robot_path`` and return a live :class:`FrontCamera`.

        Args:
            robot_path: Robot root prim; the camera goes under ``<robot_path>/base``.
            resolution: Sensor image size (WIDTH, HEIGHT). The field of view stays 73 deg horizontally
                for any size (fx scales with the width: 864.4 px at 1280).
            tick_rate: How often the sensor renders, in Hz of simulation time (0 = every frame).
            window: Also open a viewport window that shows the camera.
            window_resolution: Render size of that window (WIDTH, HEIGHT).
            depth: Also attach the ``distance_to_image_plane`` annotator (metres, for debugging/ground truth;
                the real Go2 front camera has no depth).
            window_title: Title of the viewport window (also used to find/destroy it).

        Raises:
            RuntimeError: If there is no stage, the robot base prim does not exist, or a camera prim at the
                mount path exists in a layer other than the session layer (e.g. saved into the scene file).
        """
        # Imported here so that `import go2_camera` works even before these extensions are loaded.
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("FrontCamera: no stage is open")
        base_path = f"{robot_path.rstrip('/')}/base"
        if not stage.GetPrimAtPath(base_path).IsValid():
            raise RuntimeError(f"FrontCamera: robot base prim '{base_path}' does not exist (spawn the robot first)")

        self = cls(robot_path, resolution, window_resolution, window_title, depth)

        # 1. Clean up a previous camera on the same robot (live object, or leftovers of a crashed script).
        previous = _registry().get(self.mount_path)
        if previous is not None:
            previous.close()
        self._destroy_window_by_title()
        self._remove_render_products(stage)
        self._remove_prims(stage)
        if stage.GetPrimAtPath(self.mount_path).IsValid():
            layers = [spec.layer.identifier for spec in stage.GetPrimAtPath(self.mount_path).GetPrimStack()]
            raise RuntimeError(f"FrontCamera: '{self.mount_path}' is defined in {layers}; remove it there first")

        width, height = self.resolution
        try:
            # 2. Author the prims on the SESSION layer: they are visible to physics/rendering like any prim,
            #    but File > Save does not write them into the scene file.
            with Usd.EditContext(stage, stage.GetSessionLayer()):
                mount = UsdGeom.Xform.Define(stage, self.mount_path)
                mount.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*MOUNT_TRANSLATION))

                # RtxCamera = UsdGeom.Camera + OmniSensorAPI (enables omni:sensor:tickRate).
                # It also resets the xform ops to translate/orient/scale, which set_local_poses() needs.
                self.rtx_camera = RtxCamera(self.camera_path, tick_rate=float(tick_rate))
                # LOCAL pose relative to the mount (the constructor's `orientations=` would be WORLD).
                self.rtx_camera.set_local_poses(translations=[[0.0, 0.0, 0.0]],
                                                orientations=[list(CAMERA_ORIENTATION_WXYZ)])
                lens = self.rtx_camera.camera  # isaacsim.core.experimental.objects.Camera over the same prim
                # The wrapper multiplies by 10 -> pass mm / 10 so USD stores 36.0 / 20.25 / 24.31 (see docstring).
                lens.set_apertures(horizontal_apertures=HORIZONTAL_APERTURE_MM / _WRAPPER_LENS_SCALE,
                                   vertical_apertures=HORIZONTAL_APERTURE_MM * height / width / _WRAPPER_LENS_SCALE)
                lens.set_focal_lengths(FOCAL_LENGTH_MM / _WRAPPER_LENS_SCALE)
                lens.set_clipping_ranges(*CLIPPING_RANGE_M)
                # Stop viewport navigation (mouse drag in the camera window) from moving the camera prim.
                prim = stage.GetPrimAtPath(self.camera_path)
                prim.CreateAttribute("omni:kit:cameraLock", Sdf.ValueTypeNames.Bool, True).Set(True)

                # 3. Sensor: a render product of (W, H) pixels + annotators that copy pixels out of it.
                #    CameraSensor wants (HEIGHT, WIDTH). It also re-derives the vertical aperture from the
                #    horizontal one so pixels stay square.
                annotators = ["rgb"] + (["distance_to_image_plane"] if depth else [])
                self.sensor = CameraSensor(self.rtx_camera, resolution=(height, width), annotators=annotators)
            self._subscribe_render_events()
        except Exception:
            self.close()  # do not leave half-made prims behind
            raise

        _registry()[self.mount_path] = self
        # 4. Optional window (a second render product of the same camera, rendered every frame while shown).
        if window:
            self.show_window()
        return self

    @staticmethod
    def live(robot_path: str = ROBOT_PATH) -> "FrontCamera | None":
        """Return the live camera created for ``robot_path`` (e.g. from another isaacsim_send call), or None."""
        return _registry().get(f"{robot_path.rstrip('/')}/base/{MOUNT_NAME}")

    # ------------------------------------------------------------------ images
    def get_rgb(self) -> np.ndarray | None:
        """Latest RGB frame as a numpy array (HEIGHT, WIDTH, 3) uint8, or None.

        None means "no frame yet": the sensor only produces frames while the timeline is playing, and
        the first one arrives a few rendered frames after create(). Frames update at ``tick_rate``.
        """
        data = self._fetch("rgb")
        if data is None:
            return None
        image = np.ascontiguousarray(data)
        if image.ndim != 3 or image.shape[2] != 3:  # defensive: CameraSensor already slices RGBA -> RGB
            image = np.ascontiguousarray(image.reshape(self.resolution[1], self.resolution[0], -1)[:, :, :3])
        return image.astype(np.uint8, copy=False)

    def get_depth(self) -> np.ndarray | None:
        """Latest depth frame (HEIGHT, WIDTH) float32 in metres along the optical axis, or None (no frame yet).

        Pixels that hit nothing are ``inf``. Requires ``create(..., depth=True)``.
        """
        if not self.depth_enabled:
            raise ValueError("FrontCamera: depth was not enabled; use FrontCamera.create(..., depth=True)")
        data = self._fetch("distance_to_image_plane")
        if data is None:
            return None
        return np.ascontiguousarray(data).reshape(self.resolution[1], self.resolution[0]).astype(np.float32, copy=False)

    async def wait_for_rgb(self, max_updates: int = 120, fresh: bool = False,
                           timeout_s: float = 8.0) -> np.ndarray | None:
        """Return a copied RGB frame, or None on timeout. Never fall back to stale data.

        With fresh=True require two sensor render completions after the call, plus an
        app update for the annotator copy. The extra render avoids the preceding
        render's delayed annotation being mistaken for a fresh frame. Does not
        modify the timeline or render-product lifecycle.
        """
        app = omni.kit.app.get_app()
        start_count = self.render_count
        deadline = time.monotonic() + timeout_s
        for _ in range(max_updates):
            if time.monotonic() >= deadline:
                return None
            if not fresh or self.render_count >= start_count + 2:
                if fresh:
                    await app.next_update_async()  # the annotator copies the frame one update after rendering
                    if time.monotonic() >= deadline:
                        return None
                image = self.get_rgb()
                if image is not None:
                    return image.copy()
            await app.next_update_async()
        return None

    # NOTE: there is deliberately no pause()/resume(). Disabling the sensor render product's updates
    # (HydraTexture.set_updates_enabled(False)) destroys its GPU buffers while the rgb annotator still points
    # at them; reading a frame then raised "CUDA error 700: illegal memory access", which permanently broke
    # CUDA for the whole Isaac Sim process (the Go2 policy failed too) until restart. To save FPS between
    # inspection points, close() the camera and create() it again (~0.1-0.2 s, first frame ~5 updates later).

    @property
    def render_count(self) -> int:
        """How many frames the sensor has rendered since create() (counts render-completion events)."""
        return self._render_count

    def save_png(self, path: str, image: np.ndarray | None = None, *, overwrite: bool = False) -> bool:
        """Save the supplied RGB array, or latest buffer when omitted. Refuse overwrite by default."""
        if image is None:
            image = self.get_rgb()
        if image is None:
            return False
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("expected RGB uint8 HxWx3")
        folder = os.path.dirname(os.path.abspath(path))
        os.makedirs(folder, exist_ok=True)
        with open(path, "wb" if overwrite else "xb") as stream:
            _write_png(stream, image)
        return True

    def intrinsics(self) -> dict:
        """Pinhole intrinsics of the sensor image, computed from the USD lens attributes.

        Same convention as Isaac Sim's ROS 2 CameraInfo helper (isaacsim.ros2.core camera_info_utils):
        fx = W * focal / hAperture, fy = H * focal / vAperture, cx = W / 2, cy = H / 2 (no lens distortion).

        Returns:
            dict with fx, fy, cx, cy (pixels), width, height (pixels), hfov_deg, vfov_deg.
        """
        camera = UsdGeom.Camera(omni.usd.get_context().get_stage().GetPrimAtPath(self.camera_path))
        if not camera:
            raise RuntimeError(f"FrontCamera: camera prim '{self.camera_path}' does not exist (closed?)")
        focal = camera.GetFocalLengthAttr().Get()
        h_ap = camera.GetHorizontalApertureAttr().Get()
        v_ap = camera.GetVerticalApertureAttr().Get()
        width, height = self.resolution
        return {
            "fx": width * focal / h_ap,
            "fy": height * focal / v_ap,
            "cx": width / 2.0,
            "cy": height / 2.0,
            "width": width,
            "height": height,
            "hfov_deg": math.degrees(2.0 * math.atan(h_ap / (2.0 * focal))),
            "vfov_deg": math.degrees(2.0 * math.atan(v_ap / (2.0 * focal))),
        }

    # ------------------------------------------------------------------ window
    def show_window(self) -> None:
        """Open the camera window (or make a hidden / [x]-closed one visible again).

        The window floats; the main "Viewport" stays the active viewport (see :meth:`_window_fixup`).
        """
        if self._closed:
            raise RuntimeError("FrontCamera: already closed")
        previous_active = self._active_viewport_title()
        if self.window is not None and self._window_alive():
            self.window.visible = True  # ViewportWindow's setter also re-enables viewport rendering
            self.window.viewport_api.updates_enabled = True
        else:
            from isaacsim.core.rendering_manager import ViewportManager

            self._destroy_window_by_title()  # never two windows with the same title
            # resolution here is (WIDTH, HEIGHT). This is the window's own render product of the same camera.
            self.window = ViewportManager.create_viewport_window(
                camera=self.camera_path, title=self.window_title, resolution=self.window_resolution
            )
            # When the user closes the window with its [x], stop rendering it too (saves GPU time).
            weak_self = weakref.ref(self)

            def on_visibility_changed(visible: bool) -> None:
                owner = weak_self()
                if owner is not None and owner.window is not None:
                    try:
                        owner.window.viewport_api.updates_enabled = bool(visible)
                    except Exception:  # window being destroyed
                        pass

            self.window.set_visibility_changed_fn(on_visibility_changed)
        if self._fixup_task is not None and not self._fixup_task.done():
            self._fixup_task.cancel()
        self._fixup_task = asyncio.ensure_future(self._window_fixup(previous_active))

    def hide_window(self) -> None:
        """Hide the camera window and stop rendering it (the 10 Hz sensor keeps running)."""
        if self.window is not None and self._window_alive():
            self.window.visible = False
            self.window.viewport_api.updates_enabled = False

    def is_window_visible(self) -> bool:
        return self.window is not None and self._window_alive() and bool(self.window.visible)

    # ------------------------------------------------------------------ teardown
    def close(self) -> None:
        """Remove everything this camera created. Safe to call twice; does not touch the timeline.

        Order matters: 1) the window (its render product looks through our camera prim), 2) the sensor's
        render product + annotators, 3) leftover render-product prims, then the camera prims themselves
        (removing a camera that a live render product still targets leaves the renderer pointing at nothing).
        When the stage is about to be replaced, call this first, then stop the timeline (CLAUDE.md rule).
        """
        if self._closed:
            return
        self._closed = True
        if self._fixup_task is not None and not self._fixup_task.done():
            self._fixup_task.cancel()
        self._fixup_task = None
        # 1. Window.
        if self.window is not None:
            try:
                self.window.set_visibility_changed_fn(None)
            except Exception:
                pass
        self._destroy_window_by_title()
        self.window = None
        # 2. Sensor: detach annotators and destroy the render product now (not at garbage collection).
        self._render_sub = None  # dropping the subscription object unsubscribes
        if self.sensor is not None:
            try:
                self.sensor._invalidate_sensor()
            except Exception as error:
                carb.log_warn(f"FrontCamera: releasing the sensor failed: {error!r}")
        self.sensor = None
        self.rtx_camera = None
        # 3. Prims: first the render-product prims that still point at our camera (ViewportWindow.destroy()
        #    releases its texture but leaves its RenderProduct prim behind), then the camera prims.
        stage = omni.usd.get_context().get_stage()
        if stage is not None:
            self._remove_render_products(stage)
            self._remove_prims(stage)
        reg = _registry()
        if reg.get(self.mount_path) is self:
            del reg[self.mount_path]

    # ------------------------------------------------------------------ helpers
    def _hydra_texture(self):
        """The omni.hydratexture object behind the sensor's render product (private Isaac/Replicator API:
        CameraSensor._hydra_texture is a Replicator HydraTexture wrapper, its .hydra_texture the native one)."""
        wrapper = getattr(self.sensor, "_hydra_texture", None) if self.sensor is not None else None
        return getattr(wrapper, "hydra_texture", None)

    def _subscribe_render_events(self) -> None:
        """Count the sensor's rendered frames (used by wait_for_rgb(fresh=True))."""
        import carb.eventdispatcher
        import omni.hydratexture

        texture = self._hydra_texture()
        if texture is None or not hasattr(texture, "get_event_key"):
            return
        weak_self = weakref.ref(self)

        def on_render_done(_event) -> None:
            owner = weak_self()
            if owner is not None:
                owner._render_count += 1

        self._render_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            observer_name=f"go2_camera.FrontCamera:{id(self)}",
            event_name=omni.hydratexture.GLOBAL_EVENT_DRAWABLE_CHANGED,
            filter=texture.get_event_key(),
            on_event=on_render_done,
        )

    def _fetch(self, annotator: str):
        """Get one annotator's latest data as a numpy array (copied from the GPU), or None."""
        if self.sensor is None:
            return None
        try:
            data, _info = self.sensor.get_data(annotator)
        except RuntimeError as error:  # size mismatch while the render product warms up
            carb.log_warn(f"FrontCamera: {annotator} not ready: {error}")
            return None
        if data is None:
            return None
        return data.numpy() if hasattr(data, "numpy") else np.asarray(data)

    async def _window_fixup(self, previous_active: str | None, frames: int = 15) -> None:
        """Runs for a few frames after show_window(). WHY (both observed live in Isaac Sim 6.1):

        1. ViewportWindow builds its UI one frame later and then re-applies its saved/default render
           resolution (1280x720), overriding the 640x360 that create_viewport_window() set. Re-apply ours.
        2. A newly shown window takes focus and becomes the "active viewport". Many scripts act on the
           active viewport (screenshots, set_camera_view, the keyboard needs the main viewport focused),
           so give focus back to the window that was active before.
        """
        import omni.ui as ui

        app = omni.kit.app.get_app()
        focus_target = previous_active if previous_active and previous_active != self.window_title else "Viewport"
        for _ in range(frames):  # 15 frames = 0.3 s at 50 FPS
            await app.next_update_async()
            if self._closed or not self._window_alive():
                return
            api = self.window.viewport_api
            if tuple(api.resolution) != self.window_resolution:
                api.resolution = self.window_resolution
            if self._active_viewport_title() == self.window_title:
                target = ui.Workspace.get_window(focus_target)
                if target is not None and target.visible:
                    target.focus()

    @staticmethod
    def _active_viewport_title() -> str | None:
        try:
            import omni.kit.viewport.window as viewport_window

            active = viewport_window.ViewportWindow.active_window
            return active.title if active else None
        except ReferenceError:  # the active window was destroyed
            return None

    def _window_alive(self) -> bool:
        """True if our window still exists (someone may have destroyed all viewport windows meanwhile)."""
        if self.window is None:
            return False
        try:
            import omni.kit.viewport.window as viewport_window

            return any(w.title == self.window_title for w in viewport_window.get_viewport_window_instances() if w)
        except ReferenceError:
            return False

    def _destroy_window_by_title(self) -> None:
        try:
            from isaacsim.core.rendering_manager import ViewportManager

            # include=None would destroy ALL viewport windows (also the main one) -> always pass our title.
            ViewportManager.destroy_viewport_windows(include=[re.escape(self.window_title)])
        except Exception as error:
            carb.log_warn(f"FrontCamera: destroying window '{self.window_title}' failed: {error!r}")

    def _remove_render_products(self, stage: Usd.Stage) -> None:
        """Delete RenderProduct prims under /Render whose camera is OUR camera (from every layer that has them).

        Only products targeting ``self.camera_path`` are touched, never the main viewport's or other tools'.
        Same technique as Replicator's HydraTexture.destroy() (it removes its product from all layers).
        """
        render_root = stage.GetPrimAtPath("/Render")
        if not render_root:
            return
        ours = []
        for prim in Usd.PrimRange(render_root):
            if prim.IsA(UsdRender.Product):
                targets = UsdRender.Product(prim).GetCameraRel().GetTargets()
                if any(str(t) == self.camera_path for t in targets):
                    ours.append(prim.GetPath())
        for path in ours:
            for layer in stage.GetLayerStack():  # includes the session layer
                if layer.GetPrimAtPath(path):
                    with Usd.EditContext(stage, layer):
                        stage.RemovePrim(path)

    def _remove_prims(self, stage: Usd.Stage) -> None:
        """Delete the mount (and camera) spec from the session layer and prune the empty 'over's left behind."""
        layer = stage.GetSessionLayer()
        if layer.GetPrimAtPath(self.mount_path):
            with Usd.EditContext(stage, layer):
                stage.RemovePrim(self.mount_path)
        # Defining a prim on the session layer creates `over World { over Robot { over base {...}}}`.
        # Remove those overs again, but only if they hold nothing else (another tool may use them).
        path = Sdf.Path(self.mount_path).GetParentPath()
        while path != Sdf.Path.absoluteRootPath:
            spec = layer.GetPrimAtPath(path)
            if not spec or spec.specifier != Sdf.SpecifierOver or spec.nameChildren or spec.properties \
                    or spec.ListInfoKeys() not in ([], ["specifier"]):
                break
            with Usd.EditContext(stage, layer):
                stage.RemovePrim(path)
            path = path.GetParentPath()
