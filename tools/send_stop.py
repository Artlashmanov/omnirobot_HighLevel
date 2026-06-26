#!/usr/bin/env python3
from omni_pi.can_transport import CanTransport
from omni_pi.protocol import make_stop

tx = CanTransport("can0")

try:
    msg = make_stop(seq=1)
    tx.send(msg)
    print(f"sent: id=0x{msg.arbitration_id:03X}, data={msg.data.hex(' ')}")
finally:
    tx.close()
