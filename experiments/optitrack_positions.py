import asyncio

from behaviours.obstacle_avoidance import ObstacleAvoidance

class OptitrackPositionExperiment:

    def __init__(self, robot, config=None, logger=None):
        self.robot = robot
        self.logger = logger
        self.config = config or {}

        self.running = True
        self.paused = False

        self.delta = self.config.get("delta", 1000)
        self.wheel_velocity = self.config.get("wheel_velocity", 300)

        self.turning_left = 0

        self.obstacle_avoidance = ObstacleAvoidance(wheel_velocity=self.wheel_velocity,
                                                    delta=self.delta)

    async def run(self):
        while self.running:

            if await self.robot.get_global_pose() == None:
                print("waiting")
                self.paused = True
            else:
                self.paused = False

            if self.paused:
                await self.robot.stop()
                await asyncio.sleep(0.05)
                continue

            prox = await self.robot.proximity_horizontal()

            left, right = self.obstacle_avoidance.step_motion(prox)

            await self.robot.drive(left, right)

            pose = await self.robot.get_global_pose()

            if self.logger:
                self.logger.log(
                    state={"proximity": prox, 
                           "pose.x": pose.position[0],
                           "pose.y": pose.position[1],
                           "pose.z": pose.position[2],
                           "pose.o0": pose.orientation[0],
                           "pose.o1": pose.orientation[1],
                           "pose.o2": pose.orientation[2],
                           "pose.o3": pose.orientation[3],
                           "timestamp": pose.timestamp

                           },
                    command={
                        "left_motor": left,
                        "right_motor": right,
                    },
                )

            await asyncio.sleep(0.05)

        await self.robot.stop()


    async def pause(self):
        self.paused = True

    async def resume(self):
        self.paused = False

    async def stop(self):
        self.running = False