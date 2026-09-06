"""Simulation command shaping, independent of ROS for injected-clock tests."""
import math
from dataclasses import dataclass
from numbers import Real


def finite(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def clamp(x, lo, hi):
    if not all(finite(value) for value in (x, lo, hi)) or lo > hi:
        raise ValueError("clamp requires finite values and ordered bounds")
    return max(lo, min(hi, x))


def validate_mode(mode):
    if mode not in ("modern", "prop_m"):
        raise ValueError("mode must be modern or prop_m")
    return mode


@dataclass(frozen=True)
class Limits:
    v_max: float
    w_max: float
    a_max: float
    alpha_max: float

    def __post_init__(self):
        if not all(finite(v) and v > 0 for v in vars(self).values()):
            raise ValueError("All velocity/acceleration limits must be finite and positive")


@dataclass(frozen=True)
class ShapingConfig:
    limits: Limits
    timeout: float = 0.5
    deadband_v: float = 0.005
    deadband_w: float = 0.01
    slip_enabled: bool = True
    slip_timeout: float = 0.5
    slip_threshold: float = 0.3
    slip_hard: float = 0.6
    slip_gamma: float = 0.8

    def __post_init__(self):
        if not isinstance(self.limits, Limits):
            raise ValueError("limits must be validated Limits")
        for name in ("timeout", "slip_timeout"):
            if not finite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not all(finite(v) and v >= 0 for v in (self.deadband_v, self.deadband_w)):
            raise ValueError("deadbands must be finite and nonnegative")
        if type(self.slip_enabled) is not bool:
            raise ValueError("slip_enabled must be boolean")
        if not all(finite(v) for v in (self.slip_threshold, self.slip_hard, self.slip_gamma)):
            raise ValueError("slip configuration must be finite")
        if not 0 <= self.slip_threshold < self.slip_hard <= 1 or not 0 <= self.slip_gamma <= 1:
            raise ValueError("Require 0 <= slip threshold < hard <= 1 and 0 <= gamma <= 1")


class CommandShaper:
    def __init__(self, clock):
        self.clock = clock
        self.last_update = clock()
        self.command = (0.0, 0.0)
        self.command_time = None
        self.slip = 0.0
        self.slip_time = None
        self.output = (0.0, 0.0)

    def command_received(self, v, w):
        if not all(finite(value) for value in (v, w)):
            self.command_time = None
            return False
        self.command, self.command_time = (v, w), self.clock()
        return True

    def slip_received(self, value):
        if not finite(value):
            self.slip_time = None
            return False
        self.slip, self.slip_time = value, self.clock()
        return True

    def inhibit(self):
        self.command_time = self.slip_time = None
        self.output = (0.0, 0.0)
        return self.output

    def step(self, config):
        now = self.clock()
        dt = now - self.last_update
        self.last_update = now
        if not finite(now) or not finite(dt) or dt < 0:
            return self.inhibit()
        if dt == 0:
            return self.output  # Paused ROS simulation time cannot advance ramps.
        dt = min(dt, 0.2)

        def fresh(timestamp, timeout):
            return timestamp is not None and 0 <= now - timestamp <= timeout

        v, w = self.command if fresh(self.command_time, config.timeout) else (0.0, 0.0)
        v = 0.0 if abs(v) < config.deadband_v else v
        w = 0.0 if abs(w) < config.deadband_w else w
        limits = config.limits
        v, w = clamp(v, -limits.v_max, limits.v_max), clamp(w, -limits.w_max, limits.w_max)
        if config.slip_enabled:
            if not fresh(self.slip_time, config.slip_timeout) or self.slip >= config.slip_hard:
                v = w = 0.0
            elif self.slip > config.slip_threshold:
                v, w = v * config.slip_gamma, w * config.slip_gamma
        old_v, old_w = self.output
        self.output = (old_v + clamp(v - old_v, -limits.a_max * dt, limits.a_max * dt),
                       old_w + clamp(w - old_w, -limits.alpha_max * dt, limits.alpha_max * dt))
        return self.output
