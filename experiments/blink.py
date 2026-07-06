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

            await self.robot.top_led(0, 0, 255)

            if self.logger:
                self.logger.log(
                    state={"led": "green"},
                    command={}
                )

            await asyncio.sleep(1)

            await self.robot.top_led(255, 255, 0)

            if self.logger:
                self.logger.log(
                    state={"led": "red"},
                    command={}
                )

            await asyncio.sleep(1)

        await self.robot.top_led(0, 0, 0)

    async def pause(self):
        self.paused = True

    async def resume(self):
        self.paused = False

    async def stop(self):
        self.running = False