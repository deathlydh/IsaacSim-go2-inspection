"""One explicit command owner; shared by startup and hot attachment to a live session."""
import math
import numpy as np


class CommandControl:
    def __init__(self, session):
        self.session = session
        self.source = "manual"
        self.owner = None
        self.cancel_requested = False
        self.pressed = set()
        self._timeline_sub = None

    def watch_timeline(self):
        import omni.timeline
        timeline = omni.timeline.get_timeline_interface()
        def on_timeline(event):
            if event.type == int(omni.timeline.TimelineEventType.STOP):
                self.cancel()
        self._timeline_sub = timeline.get_timeline_event_stream().create_subscription_to_pop(on_timeline)

    def acquire(self, owner):
        if self.owner is not None:
            raise RuntimeError("mission_already_running")
        self.source, self.owner = "route", owner
        self.cancel_requested = False
        self.pressed.clear()
        self.stop()

    def stop(self):
        self.session.command[:] = 0.0
        try:
            _, quat = self.session.runner.articulation.get_world_poses()
            w, x, y, z = np.asarray(quat.numpy() if hasattr(quat, "numpy") else quat).reshape(-1)[:4]
            self.session._heading_target = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        except Exception:
            self.session._heading_target = None

    def cancel(self):
        self.cancel_requested = True
        self.pressed.clear()
        self.stop()

    def release(self, owner):
        if self.owner is not owner:
            raise RuntimeError("wrong_command_owner")
        self.stop()
        self.owner, self.source = None, "manual"
        self.pressed.clear()

    def on_key(self, event, *args, **kwargs):
        import carb.input
        key = getattr(event.input, "name", None)
        press = event.type == carb.input.KeyboardEventType.KEY_PRESS
        release = event.type == carb.input.KeyboardEventType.KEY_RELEASE
        if key == "ESCAPE" and press:
            self.cancel()
        if self.source != "manual":
            return True
        if key in self.session.keymap:
            if press:
                self.pressed.add(key)
            elif release:
                self.pressed.discard(key)
            # Set membership prevents repeats/stale key-release events accumulating velocity.
            self.session.command[:] = 0.0
            for active in self.pressed:
                self.session.command += np.asarray(self.session.keymap[active], dtype=np.float32)
        return True


def install_control(session):
    """Attach once without restarting the robot, scene, timeline or camera."""
    if getattr(session, "control", None) is not None:
        if getattr(session.control, "_timeline_sub", None) is None:
            CommandControl.watch_timeline(session.control)
        guard_policy_reset(session,session.control)
        return session.control
    control = CommandControl(session)
    if session._key_sub is not None:
        session._input.unsubscribe_to_keyboard_events(session._keyboard, session._key_sub)
    session.control = control
    session._key_sub = session._input.subscribe_to_keyboard_events(session._keyboard, control.on_key)
    control.watch_timeline()
    guard_policy_reset(session,control)
    control.stop()
    return control


def guard_policy_reset(session,control):
    """Also protect a live session started by the older startup script.

    The old physics callback auto-restarted an invalid runner. Reject the reset
    before any pose changes; that callback marks the policy failed, and the
    route action exits. Normal startup/manual reset behaviour stays available.
    """
    runner = session.runner
    if runner is None or getattr(runner,"_inspection_reset_guard",False):
        return
    original_reset = runner.restart_from_default_state
    def guarded_reset(*args,**kwargs):
        if control.source == "route":
            control.stop()
            raise RuntimeError("policy_reset_forbidden_during_route")
        return original_reset(*args,**kwargs)
    runner.restart_from_default_state = guarded_reset
    runner._inspection_reset_guard = True
