import socket
import struct
import select
import threading
import time
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple


POS_SCALE = 1000.0
ORI_SCALE = 32767.0


@dataclass
class PoseSample:
    obj_id: int
    x: float
    y: float
    z: float
    ox: float
    oy: float
    oz: float
    t: float


class UdpPositionReceiver:
    def __init__(
        self,
        ports: Iterable[int] = (40001, 40002),
        listen_ip: str = "0.0.0.0",
        object_id: Optional[int] = None,
    ) -> None:
        self._ports = list(ports)
        self._listen_ip = listen_ip
        self._object_id = object_id
        self._socks = []
        self._lock = threading.Lock()
        self._latest: Optional[PoseSample] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._running:
            return
        self._socks = [self._make_sock(p) for p in self._ports]
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        for s in self._socks:
            try:
                s.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def get_latest(self) -> Optional[PoseSample]:
        with self._lock:
            return self._latest

    def set_object_id(self, object_id: Optional[int]) -> None:
        self._object_id = object_id

    def _make_sock(self, port: int) -> socket.socket:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self._listen_ip, port))
        return s

    def _run(self) -> None:
        while self._running:
            try:
                ready, _, _ = select.select(self._socks, [], [], 0.05)
            except (OSError, ValueError):
                continue

            now = time.monotonic()
            for s in ready:
                try:
                    data, _ = s.recvfrom(2048)
                except OSError:
                    continue
                if len(data) != 14:
                    continue
                try:
                    obj_id, ix, iy, iz, iox, ioy, ioz = struct.unpack("<7h", data)
                except struct.error:
                    continue

                if self._object_id is not None and obj_id != self._object_id:
                    continue

                x = ix / POS_SCALE
                y = iy / POS_SCALE
                z = iz / POS_SCALE
                ox = iox / ORI_SCALE
                oy = ioy / ORI_SCALE
                oz = ioz / ORI_SCALE

                sample = PoseSample(
                    obj_id=obj_id,
                    x=x,
                    y=y,
                    z=z,
                    ox=ox,
                    oy=oy,
                    oz=oz,
                    t=now,
                )
                with self._lock:
                    self._latest = sample


def _format_sample(sample: PoseSample) -> str:
    return (
        f"id={sample.obj_id} "
        f"pos[m]={sample.x:+.3f} {sample.y:+.3f} {sample.z:+.3f} "
        f"ori={sample.ox:+.4f} {sample.oy:+.4f} {sample.oz:+.4f}"
    )


if __name__ == "__main__":
    receiver = UdpPositionReceiver()
    receiver.start()
    print("Listening for UDP pose packets... (Ctrl+C to stop)")
    try:
        while True:
            sample = receiver.get_latest()
            if sample is not None:
                print(_format_sample(sample))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
