#!/usr/bin/env python3
import os
import sys
from omni_pi.can_transport import CanTransport
from omni_pi.protocol import make_ping, make_stop, make_motion, make_status_req, MotionMode

mode_map = {
    "forward": MotionMode.FORWARD,
    "backward": MotionMode.BACKWARD,
    "left": MotionMode.LEFT,
    "right": MotionMode.RIGHT,
    "rotate_ccw": MotionMode.ROTATE_CCW,
    "rotate_cw": MotionMode.ROTATE_CW,
    "stop_mode": MotionMode.STOP,
}

def main():
    if len(sys.argv) < 2:
        print("usage:")
        print("  python tools/send_cmd.py ping")
        print("  python tools/send_cmd.py stop")
        print("  python tools/send_cmd.py status")
        print("  python tools/send_cmd.py motion <mode> <speed>")
        sys.exit(1)

    cmd = sys.argv[1]
    tx = CanTransport(os.environ.get("CAN_IFACE", "can0"))

    try:
        if cmd == "ping":
            msg = make_ping(seq=1)
        elif cmd == "stop":
            msg = make_stop(seq=1)
        elif cmd == "status":
            msg = make_status_req(seq=1)
        elif cmd == "motion":
            if len(sys.argv) < 4:
                print("usage: python tools/send_cmd.py motion <mode> <speed>")
                sys.exit(1)
            mode_name = sys.argv[2]
            speed = int(sys.argv[3])
            if mode_name not in mode_map:
                print(f"unknown mode: {mode_name}")
                sys.exit(1)
            msg = make_motion(seq=1, mode=mode_map[mode_name], speed_pct=speed)
        else:
            print(f"unknown command: {cmd}")
            sys.exit(1)

        tx.send(msg)
        print(f"sent: id=0x{msg.arbitration_id:03X}, data={msg.data.hex(' ')}")
    finally:
        tx.close()

if __name__ == "__main__":
    main()
