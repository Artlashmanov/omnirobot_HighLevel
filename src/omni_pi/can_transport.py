import can


class CanTransport:
    def __init__(self, channel: str = "can0") -> None:
        self.bus = can.interface.Bus(channel=channel, interface="socketcan")

    def send(self, msg: can.Message) -> None:
        self.bus.send(msg)

    def recv(self, timeout: float = 1.0):
        return self.bus.recv(timeout)

    def close(self) -> None:
        self.bus.shutdown()
