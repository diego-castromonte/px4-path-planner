import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

def calculate_yaw(current_north, current_east, target_north, target_east):
    """
    Calculates the heading angle (yaw) from current position to target position.
    Returns yaw in degrees (0 to 360, where 0 is North, 90 is East).
    """
    delta_north = target_north - current_north
    delta_east = target_east - current_east

    # atan2(y, x) in NE coordinates 
    yaw_rad = math.atan2(delta_east, delta_north)

    # convert to degrees 
    yaw_deg = math.degrees(yaw_rad)

    # normalize to 0 to 360 
    if yaw_deg < 0:
        yaw_deg += 360

    return yaw_deg

async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(5)  # Wait for takeoff to complete

    waypoints = [
        (10.0, 0.0, -5.0),   # 10m North
        (10.0, 10.0, -5.0),  # 10m North, 10m East
        (0.0, 10.0, -5.0),   # 0m North, 10m East
        (0.0, 0.0, -5.0)     # Back to start
    ]

    # Initialize current simulated positions
    curr_n, curr_e, curr_d = 0.0, 0.0, -5.0

    print("setting inital offboard setpoint")
    # offboard mode requires a setpoint before deploying 

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("Disarming...")
        await drone.action.disarm()
        return

    print("Offboard mode start.")

    for target_n, target_e, target_d in waypoints: 
        print(f"Moving to North{target_n}, East={target_e}")

        # Calculate the heading needed to look at the new waypoint 
        target_yaw = calculate_yaw(curr_n, curr_e, target_n, target_e)
        print(f"Adjusting heading to face waypoint: {target_yaw} degrees")

        # step towards the waypoint while maintain yaw 
        steps = 50
        for i in range(steps):
            # position 
            curr_n += (target_n - curr_n) / (steps - i)
            curr_e += (target_e - curr_e) / (steps - i)

            await drone.offboard.set_position_ned( 
                PositionNedYaw(curr_n, curr_e, target_d, target_yaw)
            )
            await asyncio.sleep(0.1) # this is the 10Hz loop rate

    print("Path complete. Stopping offboard mode")
    await drone.offboard.stop()

    print("Landing")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())