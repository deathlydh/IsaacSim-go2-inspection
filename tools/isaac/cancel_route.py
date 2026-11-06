"""Cancel current route without changing scene or timeline."""
import sys
s = getattr(sys,"_go2_inspection_session",None)
control = getattr(s,"control",None)
if control is None:
    print("no route control installed")
else:
    control.cancel()
    print("cancel requested; command:",s.command.tolist(),"source:",control.source)
