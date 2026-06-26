#!/usr/bin/env python3
from omni_pi.can_transport import CanTransport
from omni_pi.protocol import decode_message

rx = CanTransport("can0")

print("Listening on can0... Press Ctrl+C to stop.")

try:
    while True:
        msg = rx.recv(timeout=1.0)
        if msg is not None:
            print(decode_message(msg))
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    rx.close()
