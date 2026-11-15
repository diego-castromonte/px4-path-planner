import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.telemetry import LandedState


async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for PX4 connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    print("Waiting for GPS/home position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Health OK")
            break

    # Replace HOME_LAT and HOME_LON with your QGC home coordinates.
    home_lat = 47.398000
    home_lon = 8.546000

    mission_items = [
        MissionItem(
            home_lat, home_lon, 10, -1, True,
            float("nan"), float("nan"),
            MissionItem.CameraAction.NONE,
            -1, 0, 1, float("nan"), 0,
            MissionItem.VehicleAction.NONE,
        ),
        MissionItem(
            home_lat + 0.0001, home_lon, 15, -1, True,
            float("nan"), float("nan"),
            MissionItem.CameraAction.NONE,
            -1, 0, 1, float("nan"), 0,
            MissionItem.VehicleAction.NONE,
        ),
        MissionItem(
            home_lat + 0.0001, home_lon + 0.0001, 20, -1, False,
            float("nan"), float("nan"),
            MissionItem.CameraAction.NONE,
            5, 0, 1, float("nan"), 0,
            MissionItem.VehicleAction.NONE,
        ),
    ]

    plan = MissionPlan(mission_items)
    await drone.mission.upload_mission(plan)
    print("Mission uploaded")

    await drone.action.arm()
    await drone.mission.start_mission()
    print("Mission started")

    async for progress in drone.mission.mission_progress():
        print(f"Waypoint {progress.current} of {progress.total}")

        if progress.total > 0 and progress.current == progress.total:
            break

    await drone.action.return_to_launch()
    print("Returning to launch")

    async for state in drone.telemetry.landed_state():
        if state == LandedState.ON_GROUND:
            break

    print("Landed")


if __name__ == "__main__":
    asyncio.run(main())