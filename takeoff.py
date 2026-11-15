import asyncio
from mavsdk import System


async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for PX4 connection...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("PX4 connected.")
            break

    print("Setting takeoff altitude to 3 m...")
    await drone.action.set_takeoff_altitude(3.0)

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()

    await asyncio.sleep(8)

    print("Landing...")
    await drone.action.land()

    await asyncio.sleep(8)
    print("Test complete.")


if __name__ == "__main__":
    asyncio.run(main())