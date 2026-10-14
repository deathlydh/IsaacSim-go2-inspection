import omni.timeline

tl = omni.timeline.get_timeline_interface()
tl.pause()
print("playing:", tl.is_playing())
