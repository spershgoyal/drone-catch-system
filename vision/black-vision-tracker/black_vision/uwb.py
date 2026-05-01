from __future__ import annotations

import re
import time
from dataclasses import dataclass

try:
    import serial
except ModuleNotFoundError:  # pragma: no cover - exercised only on hardware runs
    serial = None


@dataclass(slots=True)
class AnchorMeasurement:
    anchor_id: str
    anchor_address: str
    tag_address: str
    payload: str
    payload_length: int
    distance_m: float
    rssi: float | None
    timestamp_s: float
    raw_line: str


def parse_anchor_rcv_line(
    line: str,
    *,
    anchor_id: str,
    anchor_address: str,
    timestamp_s: float | None = None,
) -> AnchorMeasurement | None:
    stripped = line.strip()
    if not stripped.startswith("+ANCHOR_RCV"):
        return None

    _, separator, payload = stripped.partition("=")
    if not separator:
        return None

    parts = [part.strip() for part in payload.split(",")]
    if len(parts) < 2:
        return None

    tag_address = parts[0]
    message_payload = ""
    payload_length = 0
    distance_field = ""
    rssi_field = ""

    if len(parts) >= 4 and parts[1].isdigit():
        payload_length = int(parts[1])
        message_payload = parts[2]
        distance_field = parts[3]
        rssi_field = parts[4] if len(parts) >= 5 else ""
    else:
        distance_field = parts[1]
        rssi_field = parts[2] if len(parts) >= 3 else ""

    distance_cm = _parse_distance_cm(distance_field)
    if distance_cm is None:
        return None

    return AnchorMeasurement(
        anchor_id=anchor_id,
        anchor_address=anchor_address,
        tag_address=tag_address,
        payload=message_payload,
        payload_length=payload_length,
        distance_m=distance_cm / 100.0,
        rssi=_parse_rssi(rssi_field),
        timestamp_s=time.monotonic() if timestamp_s is None else timestamp_s,
        raw_line=stripped,
    )


def _parse_distance_cm(value: str) -> float | None:
    distance_match = re.search(r"-?\d+(?:\.\d+)?", value)
    if distance_match is None:
        return None
    return float(distance_match.group(0))


def _parse_rssi(value: str) -> float | None:
    rssi_match = re.search(r"-?\d+(?:\.\d+)?", value)
    if rssi_match is None:
        return None
    return float(rssi_match.group(0))


class ReyaxSerialAnchor:
    """Helper for a REYAX RYUW122 anchor exposed over UART."""

    def __init__(
        self,
        *,
        anchor_id: str,
        anchor_address: str,
        port: str,
        baud_rate: int = 115200,
        timeout_s: float = 0.02,
    ) -> None:
        self.anchor_id = anchor_id
        self.anchor_address = anchor_address
        self.port = port
        self.baud_rate = baud_rate
        self.timeout_s = timeout_s
        self._serial = None

    def open(self) -> None:
        if serial is None:  # pragma: no cover - depends on local environment
            raise RuntimeError(
                "pyserial is required for live UWB access. Install it with `pip install pyserial`."
            )
        self._serial = serial.Serial(self.port, self.baud_rate, timeout=self.timeout_s)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def is_open(self) -> bool:
        return bool(self._serial and self._serial.is_open)

    def send_command(self, command: str) -> None:
        if not self.is_open():
            raise RuntimeError(f"Anchor {self.anchor_id} is not open on {self.port}.")
        wire = command.strip().encode("ascii") + b"\r\n"
        self._serial.write(wire)
        self._serial.flush()

    def read_lines_until(
        self,
        *,
        timeout_s: float,
        stop_prefixes: tuple[str, ...] = ("+OK", "+ERR", "+ANCHOR_RCV"),
    ) -> list[str]:
        if not self.is_open():
            raise RuntimeError(f"Anchor {self.anchor_id} is not open on {self.port}.")

        deadline = time.monotonic() + timeout_s
        lines: list[str] = []
        while time.monotonic() < deadline:
            raw = self._serial.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue
            lines.append(line)
            if line.startswith(stop_prefixes):
                if line.startswith("+ANCHOR_RCV") or line.startswith("+ERR"):
                    break
        return lines

    def expect_ok(self, command: str, *, timeout_s: float = 0.25) -> list[str]:
        self.send_command(command)
        lines = self.read_lines_until(timeout_s=timeout_s, stop_prefixes=("+OK", "+ERR"))
        if any(line.startswith("+ERR") for line in lines):
            raise RuntimeError(f"Anchor {self.anchor_id} command failed: {lines[-1]}")
        if not any(line.startswith("+OK") for line in lines):
            raise RuntimeError(f"Anchor {self.anchor_id} timed out waiting for +OK: {command}")
        return lines

    def configure(
        self,
        *,
        network_id: str,
        password: str,
        channel: int,
        bandwidth: int,
        enable_rssi: bool,
    ) -> None:
        self.expect_ok("AT")
        self.expect_ok("AT+MODE=1")
        self.expect_ok(f"AT+ADDRESS={self.anchor_address}")
        self.expect_ok(f"AT+NETWORKID={network_id}")
        self.expect_ok(f"AT+CPIN={password}")
        self.expect_ok(f"AT+CHANNEL={channel}")
        self.expect_ok(f"AT+BANDWIDTH={bandwidth}")
        self.expect_ok(f"AT+RSSI={1 if enable_rssi else 0}")

    def poll_distance(
        self,
        *,
        tag_address: str,
        payload: str,
        send_mode: str,
        response_timeout_s: float,
    ) -> AnchorMeasurement | None:
        if send_mode == "address_only":
            command = f"AT+ANCHOR_SEND={tag_address}"
        else:
            payload_ascii = payload.encode("ascii", errors="ignore").decode("ascii")
            payload_ascii = payload_ascii[:12]
            command = f"AT+ANCHOR_SEND={tag_address},{len(payload_ascii)},{payload_ascii}"
        self.send_command(command)

        deadline = time.monotonic() + response_timeout_s
        while time.monotonic() < deadline:
            raw = self._serial.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue
            measurement = parse_anchor_rcv_line(
                line,
                anchor_id=self.anchor_id,
                anchor_address=self.anchor_address,
            )
            if measurement is not None:
                return measurement
            if line.startswith("+ERR"):
                raise RuntimeError(f"Anchor {self.anchor_id} command failed: {line}")
        return None
