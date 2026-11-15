import asyncio
import math

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


obstacle = {"north": 15.0, "east": 0.0, "radius": 4.0}
avoiding = False


async def position_monitor(drone):
    global avoiding

    async for pos_vel in drone.telemetry.position_velocity_ned():
        north = pos_vel.position.north_m
        east = pos_vel.position.east_m

        distance = math.sqrt(
            (north - obstacle["north"]) ** 2
            + (east - obstacle["east"]) ** 2
        )

        if not avoiding and distance < obstacle["radius"]:
            print(f"OBSTACLE DETECTED — distance = {distance:.1f} m")
            avoiding = True

        elif avoiding and distance > obstacle["radius"] * 1.5:
            print(f"CLEAR — distance = {distance:.1f} m")
            avoiding = False


async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    print("Waiting for GPS/home position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Ready!")
            break

    print("Arming...")
    await drone.action.arm()

    # NED: north, east, down. -10 means 10 m above home.
    print("Setting initial setpoint...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0, 0, -10, 0)
    )

    print("Starting Offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard start failed: {error}")
        await drone.action.disarm()
        return

    asyncio.create_task(position_monitor(drone))

    print("Flying toward obstacle zone...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(30, 0, -10, 0)
    )

    for _ in range(300):  # 60 seconds total
        await asyncio.sleep(0.2)

        if avoiding:
            # Detour 8 m east around the obstacle.
            await drone.offboard.set_position_ned(
                PositionNedYaw(15, 8, -10, 0)
            )
        else:
            # Continue toward final target.
            await drone.offboard.set_position_ned(
                PositionNedYaw(30, 0, -10, 0)
            )

    print("Stopping Offboard mode...")
    await drone.offboard.stop()

    print("Returning to launch...")
    await drone.action.return_to_launch()


if __name__ == "__main__":
    asyncio.run(run())