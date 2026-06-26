#!/usr/bin/env python3
import sys
from omni_pi.can_transport import CanTransport
from omni_pi.protocol import make_motion, MotionMode

mode_map = {
    "forward": MotionMode.FORWARD,
    "backward": MotionMode.BACKWARD,
    "left": MotionMode.LEFT,
    "right": MotionMode.RIGHT,
    "rotate_ccw": MotionMode.ROTATE_CCW,
    "rotate_cw": MotionMode.ROTATE_CW,
    "stop": MotionMode.STOP,
}

if len(sys.argv) < 3:
    print("usage: python tools/send_motion.py <mode> <speed>")
    sys.exit(1)

mode = sys.argv[1]
speed = int(sys.argv[2])

if mode not in mode_map:
    print("unknown mode")
    sys.exit(1)

tx = CanTransport("can0")

try:
    msg = make_motion(seq=1, mode=mode_map[mode], speed_pct=speed)
    tx.send(msg)
    print(f"sent: id=0x{msg.arbitration_id:03X}, data={msg.data.hex(' ')}")
finally:
    tx.close()
