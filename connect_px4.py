import asyncio
from mavsdk import System


async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for PX4 connection...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"PX4 connected — system UUID: {state.uuid}")
            break

    async for position in drone.telemetry.position():
        print(f"Relative altitude: {position.relative_altitude_m:.2f} m")
        break


if __name__ == "__main__":
    asyncio.run(main())