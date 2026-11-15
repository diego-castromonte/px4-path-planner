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

    async for data in drone.telemetry.position_velocity_ned():
        position = data.position
        print(
            f"Local NED position:\n"
            f"  North: {position.north_m:.2f} m\n"
            f"  East:  {position.east_m:.2f} m\n"
            f"  Down:  {position.down_m:.2f} m"
        )
        break


if __name__ == "__main__":
    asyncio.run(main())