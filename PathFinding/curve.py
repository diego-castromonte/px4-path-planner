import math
import numpy as np


# ---------- SETTINGS ----------
DT = 0.1                    # 10 Hz command stream
CRUISE_SPEED = 3.0          # m/s on straight sections
TURN_SPEED = 1.0            # m/s at sharp turns
START_SPEED = 0.5           # m/s
MAX_ACCEL = 2.0             # m/s^2
PATH_SAMPLE_SPACING = 0.05  # meters; internal path resolution
TURN_THRESHOLD_DEG = 10.0   # ignore tiny bends


def distance(a, b):
    return float(np.linalg.norm(b - a))


def point_at_distance(route, cumulative_distances, s):
    """
    Returns the N/E point located s meters along the polyline route.
    """

    if s <= 0:
        return route[0]

    if s >= cumulative_distances[-1]:
        return route[-1]

    segment_index = np.searchsorted(cumulative_distances, s) - 1
    segment_index = max(0, segment_index)

    start_distance = cumulative_distances[segment_index]
    end_distance = cumulative_distances[segment_index + 1]

    fraction = (s - start_distance) / (end_distance - start_distance)

    return (
        route[segment_index]
        + fraction * (route[segment_index + 1] - route[segment_index])
    )


def turn_angle_degrees(a, b, c):
    """
    0 degrees = straight.
    90 degrees = right-angle turn.
    """

    incoming = b - a
    outgoing = c - b

    incoming_length = np.linalg.norm(incoming)
    outgoing_length = np.linalg.norm(outgoing)

    if incoming_length == 0 or outgoing_length == 0:
        return 0.0

    cosine_angle = np.dot(incoming, outgoing) / (
        incoming_length * outgoing_length
    )

    cosine_angle = np.clip(cosine_angle, -1.0, 1.0)
    return math.degrees(math.acos(cosine_angle))


def make_speed_profile(route):
    """
    Creates a physically reachable speed profile along a route.
    """

    # Find cumulative distance at each original waypoint.
    segment_lengths = [
        distance(route[i], route[i + 1])
        for i in range(len(route) - 1)
    ]

    waypoint_distances = [0.0]
    for segment_length in segment_lengths:
        waypoint_distances.append(
            waypoint_distances[-1] + segment_length
        )

    total_distance = waypoint_distances[-1]

    # Create many evenly spaced path samples.
    sample_distances = np.arange(
        0.0,
        total_distance,
        PATH_SAMPLE_SPACING
    )

    if sample_distances[-1] != total_distance:
        sample_distances = np.append(sample_distances, total_distance)

    # Start by allowing full cruise speed everywhere.
    speed_limits = np.ones(len(sample_distances)) * CRUISE_SPEED

    # Start and final stop requirements.
    speed_limits[0] = START_SPEED
    speed_limits[-1] = 0.0

    # Detect turns and impose a lower speed limit at those locations.
    for i in range(1, len(route) - 1):
        angle = turn_angle_degrees(
            route[i - 1],
            route[i],
            route[i + 1]
        )

        if angle > TURN_THRESHOLD_DEG:
            corner_distance = waypoint_distances[i]
            sample_index = np.argmin(
                np.abs(sample_distances - corner_distance)
            )

            speed_limits[sample_index] = TURN_SPEED

            print(
                f"Turn: N={route[i][0]:.1f}, E={route[i][1]:.1f} | "
                f"Angle={angle:.1f}° | "
                f"Speed limit={TURN_SPEED:.1f} m/s"
            )

    # Backward pass:
    # Limit speed now so the virtual drone can brake before a slow future point.
    speed_profile = speed_limits.copy()

    for i in range(len(speed_profile) - 2, -1, -1):
        ds = sample_distances[i + 1] - sample_distances[i]

        max_safe_speed = math.sqrt(
            speed_profile[i + 1] ** 2 + 2 * MAX_ACCEL * ds
        )

        speed_profile[i] = min(
            speed_profile[i],
            max_safe_speed
        )

    # Forward pass:
    # Limit speed so it cannot accelerate faster than MAX_ACCEL.
    for i in range(1, len(speed_profile)):
        ds = sample_distances[i] - sample_distances[i - 1]

        max_reachable_speed = math.sqrt(
            speed_profile[i - 1] ** 2 + 2 * MAX_ACCEL * ds
        )

        speed_profile[i] = min(
            speed_profile[i],
            max_reachable_speed
        )

    return waypoint_distances, sample_distances, speed_profile


def run_trajectory_prototype():
    # Replace this later with your Smoothed North/East waypoints.
    route = np.array([
        [0.0, 0.0],
        [5.0, 0.0],
        [10.0, 0.0],
        [15.0, 0.0],   # 90-degree turn
        [15.0, 5.0],
        [15.0, 10.0],
        [15.0, 15.0],  # final destination
    ])

    print("--- BUILDING TRAJECTORY ---")

    waypoint_distances, sample_distances, speed_profile = (
        make_speed_profile(route)
    )

    total_distance = waypoint_distances[-1]

    print(f"\nRoute length: {total_distance:.2f} m")
    print("--- STREAMING 10 Hz VIRTUAL DRONE ---")

    route_progress = 0.0
    current_speed = START_SPEED
    time_s = 0.0

    while route_progress < total_distance:
        # Desired speed at the drone's current location along the route.
        desired_speed = float(
            np.interp(
                route_progress,
                sample_distances,
                speed_profile
            )
        )

        # Drone cannot instantaneously change speed.
        if current_speed < desired_speed:
            current_speed = min(
                current_speed + MAX_ACCEL * DT,
                desired_speed
            )
        else:
            current_speed = max(
                current_speed - MAX_ACCEL * DT,
                desired_speed
            )

        # Advance the virtual drone along the path.
        route_progress += current_speed * DT
        route_progress = min(route_progress, total_distance)

        position = point_at_distance(
            route,
            waypoint_distances,
            route_progress
        )

        time_s += DT

        print(
            f"Time {time_s:5.1f}s | "
            f"N={position[0]:6.2f}, E={position[1]:6.2f} | "
            f"Target speed={desired_speed:4.2f} m/s | "
            f"Virtual speed={current_speed:4.2f} m/s"
        )

    print("\nRoute complete.")


if __name__ == "__main__":
    run_trajectory_prototype()