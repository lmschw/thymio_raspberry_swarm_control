"""
Best-of-Two obstacle avoidance for Thymio (Raspberry Pi 5 host).

Python/asyncio port of CThymioBestOfTwo::StepMotion (ARGoS reference
implementation). Drives any object exposing `async def drive(left, right)`
and `async def proximity_horizontal() -> list[float]` — i.e. your Robot class.
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass


@dataclass
class AvoidanceConfig:
    wheel_velocity: float = 200.0
    delta: float = 0.15                 # obstacle-detection threshold, normalized [0, 1]
    turning_steps: int = 8              # K: steps to keep turning once triggered
    straight_angle_range: tuple[float, float] = (-0.3, 0.3)  # radians, "go straight" cone
    sensor_max_value: float = 4500.0    # raw Thymio prox range (~0-4500); used to normalize

    # Angles (radians) of the 7 horizontal proximity sensors:
    # front-left-outer, front-left-inner, front-center, front-right-inner,
    # front-right-outer, rear-right, rear-left. Verify/calibrate against your unit.
    sensor_angles: tuple[float, ...] = (
        math.radians(45),
        math.radians(20),
        math.radians(0),
        math.radians(-20),
        math.radians(-45),
        math.radians(-142),
        math.radians(142),
    )


class BestOfTwoAvoidance:
    """Stateful obstacle-avoidance controller. Call `step()` in a loop."""

    def __init__(self, robot, config=None, logger=None):
        self.robot = robot
        self.config = config or AvoidanceConfig()
        self.logger = logger
        self._turning_left_steps = 0  # mirrors m_unTurningLeft

    def _normalize(self, readings: list[float]) -> list[float]:
        m = self.config.sensor_max_value
        return [min(max(r / m, 0.0), 1.0) for r in readings]

    async def step(self, readings: list[float]) -> bool:
        """
        One control cycle given the 7 raw prox.horizontal readings.
        Returns True while avoiding/turning, False while driving straight
        (mirrors the C++ StepMotion return value).
        """
        cfg = self.config
        r = self._normalize(readings)

        f_front = max(r[0], r[1], r[2], r[3], r[4])
        f_rear = max(r[5], r[6])
        f_left = max(r[0], r[1], r[6])
        f_right = max(r[3], r[4], r[5])

        b_front = f_front > cfg.delta
        b_rear = f_rear > cfg.delta

        v = cfg.wheel_velocity

        # Still committed to a turn from a previous trigger
        if self._turning_left_steps > 0:
            self._turning_left_steps -= 1
            if f_left > f_right:
                await self.robot.drive(v, 0)
            else:
                await self.robot.drive(0, v)
            return True

        # Obstacle ahead only -> back-turn away from it
        if b_front and not b_rear:
            self._turning_left_steps = cfg.turning_steps
            if f_left > f_right:
                await self.robot.drive(-v, 0)
            else:
                await self.robot.drive(0, -v)
            return True

        # Obstacle behind only -> turn forward away from it
        if b_rear and not b_front:
            self._turning_left_steps = cfg.turning_steps
            if f_left > f_right:
                await self.robot.drive(v, 0)
            else:
                await self.robot.drive(0, v)
            return True

        # Obstacles on both sides -> spin in place
        if b_front and b_rear:
            self._turning_left_steps = cfg.turning_steps
            if f_left > f_right:
                await self.robot.drive(-v, v)
            else:
                await self.robot.drive(v, -v)
            return True

        # No hard trigger: steer using the vector sum of all readings
        acc_x = acc_y = 0.0
        for value, angle in zip(r, cfg.sensor_angles):
            acc_x += value * math.cos(angle)
            acc_y += value * math.sin(angle)
        n = len(r)
        acc_x /= n
        acc_y /= n

        length = math.hypot(acc_x, acc_y)
        angle = math.atan2(acc_y, acc_x)

        lo, hi = cfg.straight_angle_range
        within_cone = lo <= angle <= hi

        if not (within_cone and length < cfg.delta):
            if angle < 0:
                await self.robot.drive(v, 0)
            else:
                await self.robot.drive(0, v)
            return True

        await self.robot.drive(v, v)
        return False

    async def run(
        self,
        poll_interval: float = 0.05,
        stop_event: asyncio.Event | None = None,
    ) -> None:
        """Continuously polls the robot's prox sensors and runs step()."""
        while self.running:
            readings = await self.robot.proximity_horizontal()
            await self.step(readings)
            await asyncio.sleep(poll_interval)

    async def pause(self):
        self.paused = True

    async def resume(self):
        self.paused = False

    async def stop(self):
        self.running = False


# --- Example usage -----------------------------------------------------
#
# from robot.core.robot import Robot
# from robot.core.obstacle_avoidance import BestOfTwoAvoidance
#
# async def main():
#     async with Robot() as robot:
#         avoider = BestOfTwoAvoidance(robot)
#         await avoider.run()
#
# asyncio.run(main())
