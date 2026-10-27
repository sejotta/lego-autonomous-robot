"""Thin hardware adapter around the BrickPi3 Python library.

The rest of the project depends only on the small :class:`Hardware` protocol,
which keeps the control code importable (and testable) on a laptop without the
BrickPi3 package installed.
"""

from __future__ import annotations

import math
import time
from typing import Protocol


class HardwareError(RuntimeError):
    """Base class for hardware failures."""


class HardwareUnavailable(HardwareError):
    """Raised when the BrickPi3 library or board is not available."""


class SensorReadError(HardwareError):
    """Raised when a sensor cannot be read."""


class Hardware(Protocol):
    """Minimal interface used by the robot and odometry code."""

    def set_motor_dps(self, port: str, dps: float) -> None: ...

    def get_encoder(self, port: str) -> float: ...

    def reset_encoder(self, port: str) -> None: ...

    def set_sensor_type(self, port: str, sensor_type: str) -> None: ...

    def read_ultrasonic(self, port: str) -> float: ...

    def read_gyro(self, port: str) -> tuple[float, float]: ...

    def read_light(self, port: str) -> float: ...

    def reset_all(self) -> None: ...


class BrickPiHardware:
    """Concrete adapter for the physical BrickPi3 board."""

    _VALID_PORTS = set("ABCD1234")

    def __init__(self) -> None:
        try:
            import brickpi3  # type: ignore[import-not-found]
        except ImportError as exc:  # pragma: no cover - depends on host
            raise HardwareUnavailable(
                "The 'brickpi3' package is not installed. Install it on the "
                "Raspberry Pi (see README) or run the offline demo instead."
            ) from exc

        try:
            self._bp = brickpi3.BrickPi3()
        except Exception as exc:  # pragma: no cover - depends on host
            raise HardwareUnavailable(f"could not initialise BrickPi3: {exc}") from exc

    # ------------------------------------------------------------------
    # Low-level helpers
    # ------------------------------------------------------------------
    def _port(self, port: str) -> int:
        if port not in self._VALID_PORTS:
            raise ValueError(f"invalid BrickPi port {port!r}")
        return getattr(self._bp, f"PORT_{port}")

    # ------------------------------------------------------------------
    # Hardware protocol
    # ------------------------------------------------------------------
    def set_motor_dps(self, port: str, dps: float) -> None:
        if not math.isfinite(dps):
            dps = 0.0
        try:
            self._bp.set_motor_dps(self._port(port), dps)
        except Exception as exc:  # pragma: no cover - depends on host
            raise HardwareError(f"motor {port} write failed: {exc}") from exc

    def get_encoder(self, port: str) -> float:
        try:
            return float(self._bp.get_motor_encoder(self._port(port)))
        except Exception as exc:  # pragma: no cover - depends on host
            raise SensorReadError(f"encoder {port} read failed: {exc}") from exc

    def reset_encoder(self, port: str) -> None:
        try:
            self._bp.offset_motor_encoder(self._port(port), self._bp.get_motor_encoder(self._port(port)))
        except Exception as exc:  # pragma: no cover - depends on host
            raise HardwareError(f"encoder {port} reset failed: {exc}") from exc

    def set_sensor_type(self, port: str, sensor_type: str) -> None:
        try:
            sensor = getattr(self._bp.SENSOR_TYPE, sensor_type)
            self._bp.set_sensor_type(self._port(port), sensor)
        except Exception as exc:  # pragma: no cover - depends on host
            raise HardwareError(
                f"could not configure sensor {port} as {sensor_type}: {exc}"
            ) from exc

    def read_ultrasonic(self, port: str) -> float:
        try:
            return float(self._bp.get_sensor(self._port(port)))
        except Exception as exc:  # pragma: no cover - depends on host
            raise SensorReadError(f"ultrasonic {port} read failed: {exc}") from exc

    def read_gyro(self, port: str) -> tuple[float, float]:
        try:
            angle_deg, rate_dps = self._bp.get_sensor(self._port(port))
            return float(angle_deg), float(rate_dps)
        except Exception as exc:  # pragma: no cover - depends on host
            raise SensorReadError(f"gyro {port} read failed: {exc}") from exc

    def read_light(self, port: str) -> float:
        try:
            return float(self._bp.get_sensor(self._port(port)))
        except Exception as exc:  # pragma: no cover - depends on host
            raise SensorReadError(f"light sensor {port} read failed: {exc}") from exc

    def reset_all(self) -> None:
        try:
            self._bp.reset_all()
        except Exception:  # pragma: no cover - best effort shutdown
            pass


def settle(seconds: float = 0.5) -> None:
    """Give the BrickPi firmware time to apply a sensor configuration."""

    time.sleep(seconds)
