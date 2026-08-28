"""UDP + TCP receiver with seq / loss / rate statistics.

USB + adb reverse only forwards TCP, so the PICO app sends newline-delimited
JSON over TCP to 127.0.0.1. WiFi still uses one JSON UDP datagram.
"""

from __future__ import annotations

import logging
import select
import socket
import time
from dataclasses import dataclass

from protocol.pico_packet import PicoTeleopPacket

logger = logging.getLogger(__name__)


@dataclass
class ReceiverStats:
    receive_count: int = 0
    invalid_count: int = 0
    last_seq: int | None = None
    packet_loss: int = 0
    out_of_order: int = 0
    receive_rate_hz: float = 0.0
    last_packet_monotonic: float | None = None
    last_source: str = ""

    def seconds_since_last(self, now: float | None = None) -> float | None:
        if self.last_packet_monotonic is None:
            return None
        if now is None:
            now = time.monotonic()
        return now - self.last_packet_monotonic


class UdpReceiver:
    """Bind UDP and TCP on the same port. Each message is one JSON PicoTeleopPacket."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 15000,
        timeout_sec: float = 0.5,
        bufsize: int = 65535,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout_sec = timeout_sec
        self.bufsize = bufsize
        self.stats = ReceiverStats()
        self._udp: socket.socket | None = None
        self._tcp: socket.socket | None = None
        self._clients: dict[socket.socket, bytes] = {}
        self._rate_window: list[float] = []

    def bind(self) -> None:
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp.bind((self.host, self.port))
        udp.setblocking(False)
        self._udp = udp

        tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp.bind((self.host, self.port))
        tcp.listen(4)
        tcp.setblocking(False)
        self._tcp = tcp
        logger.info("[PICO] Listening UDP+TCP on %s:%s", self.host, self.port)

    def close(self) -> None:
        for client in list(self._clients):
            client.close()
        self._clients.clear()
        if self._udp is not None:
            self._udp.close()
            self._udp = None
        if self._tcp is not None:
            self._tcp.close()
            self._tcp = None

    def poll(self, timeout: float | None = None) -> PicoTeleopPacket | None:
        if self._udp is None or self._tcp is None:
            raise RuntimeError("UdpReceiver.bind() was not called")

        buffered = self._drain_tcp_buffers()
        if buffered is not None:
            return buffered

        wait = self.timeout_sec if timeout is None else float(timeout)
        sockets: list[socket.socket] = [self._udp, self._tcp, *self._clients]
        try:
            readable, _, _ = select.select(sockets, [], [], wait)
        except (ValueError, OSError):
            return None
        if not readable:
            return None

        for sock in readable:
            if sock is self._tcp:
                self._accept()
                continue
            if sock is self._udp:
                packet = self._recv_udp()
                if packet is not None:
                    return packet
                continue
            packet = self._recv_tcp(sock)
            if packet is not None:
                return packet
        return self._drain_tcp_buffers()

    def poll_latest(self, timeout: float | None = None) -> PicoTeleopPacket | None:
        """Read every pending datagram/line, keep only the newest frame."""
        packet = self.poll(timeout)
        if packet is None:
            return None
        while True:
            nxt = self.poll(0.0)
            if nxt is None:
                return packet
            packet = nxt

    def _drain_tcp_buffers(self) -> PicoTeleopPacket | None:
        for sock, buf in list(self._clients.items()):
            line, sep, rest = buf.partition(b"\n")
            if not sep:
                continue
            self._clients[sock] = rest
            try:
                peer = sock.getpeername()
                source = f"tcp:{peer[0]}:{peer[1]}"
            except OSError:
                source = "tcp:client"
            return self._decode(line, source)
        return None

    def timed_out(self, now: float | None = None) -> bool:
        age = self.stats.seconds_since_last(now)
        if age is None:
            return True
        return age > self.timeout_sec

    def _accept(self) -> None:
        assert self._tcp is not None
        try:
            client, addr = self._tcp.accept()
        except BlockingIOError:
            return
        client.setblocking(False)
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._clients[client] = b""
        logger.info("[PICO] TCP client %s:%s", addr[0], addr[1])

    def _recv_udp(self) -> PicoTeleopPacket | None:
        assert self._udp is not None
        try:
            data, addr = self._udp.recvfrom(self.bufsize)
        except BlockingIOError:
            return None
        return self._decode(data, f"udp:{addr[0]}:{addr[1]}")

    def _recv_tcp(self, sock: socket.socket) -> PicoTeleopPacket | None:
        try:
            chunk = sock.recv(self.bufsize)
        except BlockingIOError:
            return None
        except OSError:
            self._drop_client(sock)
            return None
        if not chunk:
            self._drop_client(sock)
            return None

        buf = self._clients.get(sock, b"") + chunk
        line, sep, rest = buf.partition(b"\n")
        if not sep:
            self._clients[sock] = buf[-self.bufsize :]
            return None
        self._clients[sock] = rest
        try:
            peer = sock.getpeername()
            source = f"tcp:{peer[0]}:{peer[1]}"
        except OSError:
            source = "tcp:client"
        return self._decode(line, source)

    def _drop_client(self, sock: socket.socket) -> None:
        self._clients.pop(sock, None)
        sock.close()

    def _decode(self, data: bytes, source: str) -> PicoTeleopPacket | None:
        packet = PicoTeleopPacket.from_json_bytes(data)
        now = time.monotonic()
        if packet is None:
            self.stats.invalid_count += 1
            return None
        packet.stamp_receive(source)
        self._update_stats(packet, now, packet.source_addr)
        return packet

    def _update_stats(self, packet: PicoTeleopPacket, now: float, source: str) -> None:
        stats = self.stats
        if stats.last_seq is not None:
            delta = packet.seq - stats.last_seq
            if delta <= 0:
                stats.out_of_order += 1
            elif delta > 1:
                stats.packet_loss += delta - 1
        stats.last_seq = packet.seq
        stats.receive_count += 1
        stats.last_packet_monotonic = now
        stats.last_source = source
        self._rate_window.append(now)
        cutoff = now - 1.0
        self._rate_window = [t for t in self._rate_window if t >= cutoff]
        stats.receive_rate_hz = float(len(self._rate_window))
