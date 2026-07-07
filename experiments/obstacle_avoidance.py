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
        self.wheel_velocity = self.config.get("wheel_velocity", 400)
        self.turn_steps = self.config.get("turn_steps", 8)

        self.turning_left = 0

        # Approximate Thymio proximity sensor angles (radians)
        self.sensor_angles = [
            math.radians(-70),
            math.radians(-35),
            math.radians(0),
            math.radians(35),
            math.radians(70),
            math.radians(145),
            math.radians(-145),
        ]

    async def run(self):

        while self.running:

            if self.paused:
                await self.robot.stop()
                await asyncio.sleep(0.05)
                continue

            prox = await self.robot.proximity_horizontal()

            left, right = self.step_motion(prox)

            await self.robot.drive(left, right)

            if self.logger:
                self.logger.log(
                    state={"proximity": prox},
                    command={
                        "left_motor": left,
                        "right_motor": right,
                    },
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