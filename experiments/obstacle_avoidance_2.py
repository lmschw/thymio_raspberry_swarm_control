import asyncio
import math


class ObstacleAvoidanceExperiment:

    def __init__(self, robot, config=None, logger=None):
        self.robot = robot
        self.logger = logger
        self.config = config or {}

        self.running = True
        self.paused = False

        # Parameters
        self.delta = self.config.get("delta", 1000)
        self.wheel_velocity = self.config.get("wheel_velocity", 250)
        self.turn_steps = self.config.get("turn_steps", 8)

        self.turning_left = 0

        # Approximate Thymio proximity sensor angles (radians)
        self.wheel_velocity: float = 200.0
        self.delta: float = 0.15                 # obstacle-detection threshold, normalized [0, 1]
        self.turning_steps: int = 8              # K: steps to keep turning once triggered
        self.straight_angle_range: tuple[float, float] = (-0.3, 0.3)  # radians, "go straight" cone
        self.sensor_max_value: float = 4500.0    # raw Thymio prox range (~0-4500); used to normalize

        # Angles (radians) of the 7 horizontal proximity sensors:
        # front-left-outer, front-left-inner, front-center, front-right-inner,
        # front-right-outer, rear-right, rear-left. Verify/calibrate against your unit.
        self.sensor_angles: tuple[float, ...] = (
            math.radians(45),
            math.radians(20),
            math.radians(0),
            math.radians(-20),
            math.radians(-45),
            math.radians(-142),
            math.radians(142),
        )

    def _normalize(self, readings: list[float]) -> list[float]:
        m = self.config.sensor_max_value
        return [min(max(r / m, 0.0), 1.0) for r in readings]

    async def step(self, readings: list[float]) -> bool:
        """
        One control cycle given the 7 raw prox.horizontal readings.
        Returns True while avoiding/turning, False while driving straight
        (mirrors the C++ StepMotion return value).
        """
        r = self._normalize(readings)

        f_front = max(r[0], r[1], r[2], r[3], r[4])
        f_rear = max(r[5], r[6])
        f_left = max(r[0], r[1], r[6])
        f_right = max(r[3], r[4], r[5])

        b_front = f_front > self.delta
        b_rear = f_rear > self.delta

        v = self.wheel_velocity

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
            self._turning_left_steps = self.turning_steps
            if f_left > f_right:
                await self.robot.drive(-v, 0)
            else:
                await self.robot.drive(0, -v)
            return True

        # Obstacle behind only -> turn forward away from it
        if b_rear and not b_front:
            self._turning_left_steps = self.turning_steps
            if f_left > f_right:
                await self.robot.drive(v, 0)
            else:
                await self.robot.drive(0, v)
            return True

        # Obstacles on both sides -> spin in place
        if b_front and b_rear:
            self._turning_left_steps = self.turning_steps
            if f_left > f_right:
                await self.robot.drive(-v, v)
            else:
                await self.robot.drive(v, -v)
            return True

        # No hard trigger: steer using the vector sum of all readings
        acc_x = acc_y = 0.0
        for value, angle in zip(r, self.sensor_angles):
            acc_x += value * math.cos(angle)
            acc_y += value * math.sin(angle)
        n = len(r)
        acc_x /= n
        acc_y /= n

        length = math.hypot(acc_x, acc_y)
        angle = math.atan2(acc_y, acc_x)

        lo, hi = self.straight_angle_range
        within_cone = lo <= angle <= hi

        if not (within_cone and length < self.delta):
            if angle < 0:
                await self.robot.drive(v, 0)
            else:
                await self.robot.drive(0, v)
            return True

        await self.robot.drive(v, v)
        return False


    async def run(self):

        while self.running:

            if self.paused:
                await self.robot.stop()
                await asyncio.sleep(0.05)
                continue

            prox = await self.robot.proximity_horizontal()

            self.step(prox)

            if self.logger:
                self.logger.log(
                    state={"proximity": prox},
                    command={},
                )

            await asyncio.sleep(0.05)

        await self.robot.stop()

    def step_motion(self, prox):
        """
        Translation of ARGoS StepMotion().
        Returns (left_motor, right_motor).
        """

        f_front = max(prox[:5])
        f_rear = max(prox[5], prox[6])

        f_left = max(prox[0], prox[1], prox[6])
        f_right = max(prox[3], prox[4], prox[5])

        b_front = f_front > self.delta
        b_rear = f_rear > self.delta

        # ---------------------------------------------------------
        # Continue a previously initiated turn
        # ---------------------------------------------------------
        if self.turning_left > 0:
            self.turning_left -= 1

            if f_left > f_right:
                return self.wheel_velocity, 0
            else:
                return 0, self.wheel_velocity

        # ---------------------------------------------------------
        # Obstacle only in front
        # ---------------------------------------------------------
        if b_front and not b_rear:
            self.turning_left = self.turn_steps

            if f_left > f_right:
                return -self.wheel_velocity, 0
            else:
                return 0, -self.wheel_velocity

        # ---------------------------------------------------------
        # Obstacle only in rear
        # ---------------------------------------------------------
        if b_rear and not b_front:
            self.turning_left = self.turn_steps

            if f_left > f_right:
                return self.wheel_velocity, 0
            else:
                return 0, self.wheel_velocity

        # ---------------------------------------------------------
        # Obstacles both front and rear
        # ---------------------------------------------------------
        if b_front and b_rear:
            self.turning_left = self.turn_steps

            if f_left > f_right:
                return -self.wheel_velocity, self.wheel_velocity
            else:
                return self.wheel_velocity, -self.wheel_velocity

        # ---------------------------------------------------------
        # Compute resultant proximity vector
        # ---------------------------------------------------------
        x = 0.0
        y = 0.0

        for value, angle in zip(prox, self.sensor_angles):
            x += value * math.cos(angle)
            y += value * math.sin(angle)

        x /= len(prox)
        y /= len(prox)

        length = math.hypot(x, y)
        angle = math.atan2(y, x)

        # Equivalent of:
        #
        # !(angle in straight_range && length < delta)
        #
        straight_range = math.radians(10)

        if not (-straight_range <= angle <= straight_range and
                length < self.delta):

            if angle < 0:
                return self.wheel_velocity, 0
            else:
                return 0, self.wheel_velocity

        # Drive straight
        return self.wheel_velocity, self.wheel_velocity

    async def pause(self):
        self.paused = True

    async def resume(self):
        self.paused = False

    async def stop(self):
        self.running = False