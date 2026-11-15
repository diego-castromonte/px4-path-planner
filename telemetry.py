import asyncio
import math
from datetime import datetime
from mavsdk import System


async def position_logger(drone):
    async for pos in drone.telemetry.position():
        print(
            f"{datetime.now():%H:%M:%S} [POS] "
            f"Lat: {pos.latitude_deg:.6f}, "
            f"Lon: {pos.longitude_deg:.6f}, "
            f"Alt: {pos.relative_altitude_m:.2f} m"
        )


async def attitude_logger(drone):
    async for att in drone.telemetry.attitude_euler():
        print(
            f"{datetime.now():%H:%M:%S} [ATT] "
            f"Roll: {att.roll_deg:.2f}, "
            f"Pitch: {att.pitch_deg:.2f}, "
            f"Yaw: {att.yaw_deg:.2f}"
        )


async def battery_logger(drone):
    async for battery in drone.telemetry.battery():
        print(
            f"{datetime.now():%H:%M:%S} [BAT] "
            f"Battery: {battery.remaining_percent * 100:.0f}% "
            f"{battery.voltage_v:.2f} V"
        )


async def speed_logger(drone):
    async for velocity in drone.telemetry.velocity_ned():
        horizontal_speed = math.sqrt(
            velocity.north_m_s**2 + velocity.east_m_s**2
        )
        print(
            f"{datetime.now():%H:%M:%S} [SPD] "
            f"Speed: {horizontal_speed:.2f} m/s"
        )


async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for PX4 connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("PX4 connected.")
            break

    asyncio.create_task(position_logger(drone))
    asyncio.create_task(attitude_logger(drone))
    asyncio.create_task(battery_logger(drone))
    asyncio.create_task(speed_logger(drone))

    print("Waiting for global and home position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    print("Landing...")
    await drone.action.land()
    await asyncio.sleep(10)


if __name__ == "__main__":
    asyncio.run(main())