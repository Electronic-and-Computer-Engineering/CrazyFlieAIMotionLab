# Usage:
#   python udp_pose_receiver.py --bind 0.0.0.0 --port 40001 --id 1001
#   python udp_pose_receiver.py --bind 0.0.0.0 --port 40001 --id 1001 --no-swap

import argparse
import socket
import struct
import select
import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Optional, Sequence, Union, List


POS_SCALE = 1000.0
ORI_SCALE = 32767.0
PACKET_SIZE = 14
PACKET_FORMAT = "<7h"
WARN_INTERVAL_S = 2.0


@dataclass
class Pose:
    x: float
    y: float
    z: float
    t: float
    yaw: Optional[float] = None

    def age(self, now: Optional[float] = None) -> float:
        if now is None:
            now = time.monotonic()
        return max(0.0, now - self.t)

    @property
    def staleness(self) -> float:
        return self.age()


class UdpPoseReceiver:
    def __init__(
        self,
        bind_ip: str = "0.0.0.0",
        bind_port: Union[int, Iterable[int]] = 40001,
        drone_id: int = 1001,
        *,
        swap_yz: bool = True,
    ) -> None:
        self._bind_ip = bind_ip
        if isinstance(bind_port, int):
            self._ports = [bind_port]
        else:
            self._ports = list(bind_port)
        self._drone_id = drone_id
        self._swap_yz = swap_yz

        self._socks: List[socket.socket] = []
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._latest: Optional[Pose] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._callbacks: List[Callable[[Pose], None]] = []
        self._last_warn_t = 0.0

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

    def get_latest(self, timeout: Optional[float] = None) -> Optional[Pose]:
        with self._cond:
            if self._latest is not None or timeout is None or timeout <= 0.0:
                return self._latest
            self._cond.wait(timeout=timeout)
            return self._latest

    def subscribe(self, callback: Callable[[Pose], None]) -> None:
        with self._lock:
            self._callbacks.append(callback)

    def _make_sock(self, port: int) -> socket.socket:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((self._bind_ip, port))
        return s

    def _warn(self, message: str) -> None:
        now = time.monotonic()
        if (now - self._last_warn_t) >= WARN_INTERVAL_S:
            print(f"[UDP][WARN] {message}")
            self._last_warn_t = now

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

                if len(data) != PACKET_SIZE:
                    self._warn(f"Unexpected packet size {len(data)} (expected {PACKET_SIZE}).")
                    continue

                try:
                    obj_id, ix, iy, iz, iox, ioy, ioz = struct.unpack(PACKET_FORMAT, data)
                except struct.error:
                    self._warn("Malformed UDP packet (unpack failed).")
                    continue

                if obj_id != self._drone_id:
                    continue

                x = ix / POS_SCALE
                y_in = iy / POS_SCALE
                z_in = iz / POS_SCALE

                if self._swap_yz:
                    y = z_in
                    z = y_in
                else:
                    y = y_in
                    z = z_in

                yaw = ioz / ORI_SCALE

                pose = Pose(x=x, y=y, z=z, t=now, yaw=yaw)

                callbacks: Sequence[Callable[[Pose], None]]
                with self._cond:
                    self._latest = pose
                    self._cond.notify_all()
                    callbacks = list(self._callbacks)

                for cb in callbacks:
                    try:
                        cb(pose)
                    except Exception:
                        self._warn("Pose callback raised an exception.")


def _format_pose(pose: Pose) -> str:
    yaw = pose.yaw if pose.yaw is not None else float("nan")
    return (
        f"pos[m]=({pose.x:+.3f}, {pose.y:+.3f}, {pose.z:+.3f}) "
        f"yaw={yaw:+.3f} age={pose.age():.3f}s"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="UDP pose receiver")
    parser.add_argument("--bind", dest="bind", default="0.0.0.0", help="Bind IP (default: 0.0.0.0)")
    parser.add_argument("--port", dest="port", type=int, default=40001, help="UDP port to bind")
    parser.add_argument("--id", dest="drone_id", type=int, default=1001, help="Drone/object ID")
    parser.add_argument("--no-swap", dest="swap", action="store_false", help="Disable Y/Z swap")
    parser.set_defaults(swap=True)
    args = parser.parse_args()

    receiver = UdpPoseReceiver(bind_ip=args.bind, bind_port=args.port, drone_id=args.drone_id, swap_yz=args.swap)
    receiver.start()
    print("Listening for UDP pose packets... (Ctrl+C to stop)")

    try:
        while True:
            pose = receiver.get_latest(timeout=0.5)
            if pose is not None:
                print(_format_pose(pose))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()


if __name__ == "__main__":
    main()
