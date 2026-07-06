import asyncio

class BlinkExperiment:

    def __init__(self, robot, config=None, logger=None):
        self.robot = robot
        self.config = config or {}
        self.logger = logger

        self.running = True
        self.paused = False

    async def run(self):
        while self.running:

            if self.paused:
                await self.robot.stop()
                await asyncio.sleep(0.1)
                continue

            await self.robot.drive(100, 100)

            if self.logger:
                self.logger.log(
                    state={"left_motor": 100, "right_motor": 100},
                    command={}
                )

            await asyncio.sleep(1)

            await self.robot.drive(-100, -100)

            if self.logger:
                self.logger.log(
                    state={"left_motor": -100, "right_motor": -100},
                    command={}
                )

            await asyncio.sleep(1)

        await self.robot.drive(0, 0)

    async def pause(self):
        self.paused = True

    async def resume(self):
        self.paused = False

    async def stop(self):
        self.running = False