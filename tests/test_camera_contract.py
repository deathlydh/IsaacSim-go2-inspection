"""Exercise actual camera method source with simulated render events; no Isaac imports."""
import ast
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch
import numpy as np


class CameraContractTest(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1]/"tools/isaac/go2_camera.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name == "FrontCamera")
        methods = [n for n in cls.body if getattr(n,"name",None) in ("wait_for_rgb","save_png")]
        writer = next(n for n in tree.body if getattr(n,"name",None) == "_write_png")
        import os, time, struct, zlib
        self.app = types.SimpleNamespace(next_update_async=self.update)
        self.ns = {"np":np,"os":os,"time":time,"struct":struct,"zlib":zlib,
                   "omni":types.SimpleNamespace(kit=types.SimpleNamespace(app=types.SimpleNamespace(get_app=lambda:self.app)))}
        exec(compile(ast.Module(body=[writer,*methods],type_ignores=[]),str(path),"exec"),self.ns)
        self.image = np.full((6,8,3),50,dtype=np.uint8)
        self.cam = types.SimpleNamespace(render_count=10,get_rgb=lambda:self.image)
        self.advance = False
        self.updates = 0

    async def update(self):
        self.updates += 1
        if self.advance:
            self.cam.render_count += 1

    async def test_fresh_timeout_never_returns_old_buffer(self):
        frame = await self.ns["wait_for_rgb"](self.cam,max_updates=3,fresh=True)
        self.assertIsNone(frame)
        self.assertIsNone(await self.ns["wait_for_rgb"](self.cam,fresh=True,timeout_s=0))

    async def test_fresh_frame_is_owned_copy_after_new_renders(self):
        self.advance = True
        frame = await self.ns["wait_for_rgb"](self.cam,max_updates=6,fresh=True)
        self.assertGreaterEqual(self.updates,3)
        self.assertTrue(np.array_equal(frame,self.image))
        self.image[:] = 200
        self.assertTrue(np.all(frame == 50))

    async def test_save_uses_supplied_array_and_refuses_overwrite(self):
        from PIL import Image
        self.cam.get_rgb = lambda: (_ for _ in ()).throw(AssertionError("buffer reread"))
        with tempfile.TemporaryDirectory() as folder:
            path = str(Path(folder)/"frame.png")
            self.assertTrue(self.ns["save_png"](self.cam,path,image=self.image))
            with Image.open(path) as saved:
                self.assertTrue(np.array_equal(np.asarray(saved),self.image))
            with self.assertRaises(FileExistsError):
                self.ns["save_png"](self.cam,path,image=self.image)


if __name__ == "__main__":
    unittest.main()
