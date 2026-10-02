#!/usr/bin/env python3
"""Record a CARLA Town10HD drive for the vehicleLocalization pipeline.

Two routes are planned on the OpenDRIVE road network:

* ``mapping``: a greedy cover that drives every non-junction road of the town
  at least once (outer ring and all inner roads);
* ``localization``: one lap of the outer ring in a chosen lane.

The ego vehicle is driven by CARLA's BasicAgent in synchronous mode with a
fixed 0.02 s step (50 Hz IMU and ground truth). A semantic ray-cast LiDAR that
mimics the recorded Ouster OS1-64 (64 channels, 1024 columns, +14.2/-17.7 deg,
2.0 m above the ground) produces one complete instantaneous sweep every 0.1 s.
Its per-point semantic tags are recorded as evaluation labels only; the
LiDAR has no intensity or reflectivity channel.

Everything is stored in CARLA's left-handed convention exactly as received;
``prepareCarlaDataset.py`` converts it to the right-handed frames of the
MATLAB pipeline. Outputs in ``--output``:

* ``lidar/NNNNNN.npy``: raw semantic LiDAR points (x, y, z, cos, instance, tag)
* ``ticks.csv``: per-step ground truth, controls, IMU and GNSS
* ``lidar_frames.csv``: one row per sweep with its step and sensor transform
* ``route.csv`` and ``metadata.json``
"""

import argparse
import csv
import json
import math
import queue
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

import carla

LIDAR_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("cos", "<f4"),
                        ("instance", "<u4"), ("tag", "<u4")])
FIXED_DELTA_SECONDS = 0.02
LIDAR_PERIOD_SECONDS = 0.1
LIDAR_ATTRIBUTES = {
    "channels": "64",
    "range": "120.0",
    # One complete sweep inside a single 0.02 s step; sensor_tick keeps 10 Hz.
    "rotation_frequency": str(1.0 / FIXED_DELTA_SECONDS),
    "points_per_second": str(int(64 * 1024 / FIXED_DELTA_SECONDS)),
    "upper_fov": "14.2",
    "lower_fov": "-17.7",
    "horizontal_fov": "360.0",
    "sensor_tick": str(LIDAR_PERIOD_SECONDS),
}
LIDAR_MOUNT = {"x": 0.0, "y": 0.0, "z": 2.0}
IMU_ATTRIBUTES = {
    "sensor_tick": "0.0",
    "noise_accel_stddev_x": "0.02", "noise_accel_stddev_y": "0.02", "noise_accel_stddev_z": "0.02",
    "noise_gyro_stddev_x": "0.0017", "noise_gyro_stddev_y": "0.0017", "noise_gyro_stddev_z": "0.0017",
    "noise_gyro_bias_x": "0.0", "noise_gyro_bias_y": "0.0", "noise_gyro_bias_z": "0.0005",
    "noise_seed": "20261001",
}
GNSS_ATTRIBUTES = {"sensor_tick": "0.1"}
VEHICLE_BLUEPRINT = "vehicle.lincoln.mkz"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--route", choices=["mapping", "localization"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--agents-path", type=Path, default=Path.home() / "carla" / "PythonAPI" / "carla",
                        help="Folder that contains CARLA's 'agents' package")
    parser.add_argument("--max-speed-kph", type=float, default=30.0)
    parser.add_argument("--lateral-acceleration", type=float, default=2.0,
                        help="Speed profile limit in curves (m/s^2)")
    parser.add_argument("--localization-lane", type=int, default=-1,
                        help="OpenDRIVE lane id of the outer-ring lap start")
    parser.add_argument("--localization-start-road", type=int, default=17)
    parser.add_argument("--mapping-start-road", type=int, default=17)
    parser.add_argument("--mapping-start-lane", type=int, default=-2)
    parser.add_argument("--dry-run", action="store_true", help="Plan and save the route only")
    parser.add_argument("--max-seconds", type=float, default=1200.0)
    parser.add_argument("--stop-after-seconds", type=float, default=None,
                        help="End the recording normally after this simulated time (test captures)")
    return parser.parse_args()


# ---------------------------------------------------------------- route planning

def lane_traversals(cmap, resolution=1.0):
    """Non-junction lanes as (start, end, length) in their driving direction."""
    groups = defaultdict(list)
    for wp in cmap.generate_waypoints(resolution):
        if wp.is_junction or wp.lane_type != carla.LaneType.Driving:
            continue
        groups[(wp.road_id, wp.lane_id)].append(wp)
    traversals = {}
    for key, wps in groups.items():
        wps.sort(key=lambda w: w.s)
        if len(wps) >= 2:
            a, b = wps[0].transform.location, wps[1].transform.location
            forward = wps[0].transform.get_forward_vector()
            if (b.x - a.x) * forward.x + (b.y - a.y) * forward.y < 0:
                wps = wps[::-1]
        traversals[key] = (wps[0], wps[-1], resolution * max(1, len(wps) - 1))
    return traversals


def route_length(route):
    total = 0.0
    for (a, _), (b, _) in zip(route[:-1], route[1:]):
        total += a.transform.location.distance(b.transform.location)
    return total


def append_route(route, segment):
    if route and segment and route[-1][0].transform.location.distance(segment[0][0].transform.location) < 0.5:
        segment = segment[1:]
    route.extend(segment)


def mapping_route(cmap, planner, start_road, start_lane):
    """Greedy cover of every non-junction road, starting on one outer lane."""
    traversals = lane_traversals(cmap)
    uncovered = {road for road, _ in traversals}
    start, end, _ = traversals[(start_road, start_lane)]
    route = planner.trace_route(start.transform.location, end.transform.location)
    uncovered -= {wp.road_id for wp, _ in route if not wp.is_junction}
    current = end
    while uncovered:
        best = None
        for (road, lane), (a, b, _) in traversals.items():
            if road not in uncovered:
                continue
            approach = planner.trace_route(current.transform.location, a.transform.location)
            cost = route_length(approach)
            if best is None or cost < best[0]:
                best = (cost, road, lane, b)
        _, road, lane, b = best
        segment = planner.trace_route(current.transform.location, b.transform.location)
        assert any(wp.road_id == road and wp.lane_id == lane for wp, _ in segment), (road, lane)
        uncovered -= {wp.road_id for wp, _ in segment if not wp.is_junction}
        append_route(route, segment)
        current = b
    return route


def outer_ring_route(cmap, start_road, lane, follow_option, step=2.0):
    """Follow one lane around the outer ring, keeping straight at junctions."""
    traversals = lane_traversals(cmap)
    start = traversals[(start_road, lane)][0]
    route = [(start, follow_option)]
    current = start
    travelled = 0.0
    while True:
        options = current.next(step)
        if not options:
            raise RuntimeError("Lane ends before the ring closes")
        heading = current.transform.rotation.yaw

        def turn(wp):
            # Heading change 15 m into the branch: options split with equal
            # initial headings, so the straight continuation shows only later.
            probe, walked = wp, 0.0
            while walked < 15.0:
                ahead = probe.next(step)
                if not ahead:
                    break
                walked += probe.transform.location.distance(ahead[0].transform.location)
                probe = ahead[0]
            return abs((probe.transform.rotation.yaw - heading + 180.0) % 360.0 - 180.0)

        nxt = min(options, key=turn) if len(options) > 1 else options[0]
        travelled += current.transform.location.distance(nxt.transform.location)
        route.append((nxt, follow_option))
        current = nxt
        if travelled > 200.0 and current.transform.location.distance(start.transform.location) < step:
            break
        if travelled > 3000.0:
            raise RuntimeError("Outer ring did not close")
    return route


def speed_profile(route, max_speed, lateral_acceleration, deceleration=1.5):
    """Speed limit per route point from curvature, with braking look-ahead."""
    xy = np.array([[wp.transform.location.x, wp.transform.location.y] for wp, _ in route])
    yaw = np.unwrap(np.radians([wp.transform.rotation.yaw for wp, _ in route]))
    ds = np.r_[0.0, np.hypot(*np.diff(xy, axis=0).T)]
    s = np.cumsum(ds)
    curvature = np.zeros(len(route))
    for i in range(len(route)):
        j0 = np.searchsorted(s, s[i] - 4.0)
        j1 = min(len(route) - 1, np.searchsorted(s, s[i] + 4.0))
        if s[j1] - s[j0] > 1.0:
            curvature[i] = abs(yaw[j1] - yaw[j0]) / (s[j1] - s[j0])
    speed = np.minimum(max_speed, np.sqrt(lateral_acceleration / np.maximum(curvature, 1e-6)))
    for i in range(len(route) - 2, -1, -1):
        speed[i] = min(speed[i], math.sqrt(speed[i + 1] ** 2 + 2.0 * deceleration * (s[i + 1] - s[i])))
    return s, speed, curvature


def save_route(path, route, s, speed, curvature):
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["index", "s_m", "x_carla", "y_carla", "z_carla", "yaw_carla_deg", "road_id", "lane_id",
                         "is_junction", "curvature_per_m", "speed_limit_mps"])
        for i, (wp, _) in enumerate(route):
            loc = wp.transform.location
            writer.writerow([i, f"{s[i]:.3f}", f"{loc.x:.4f}", f"{loc.y:.4f}", f"{loc.z:.4f}",
                             f"{wp.transform.rotation.yaw:.4f}", wp.road_id, wp.lane_id, int(wp.is_junction),
                             f"{curvature[i]:.5f}", f"{speed[i]:.3f}"])


# ---------------------------------------------------------------- recording

def spawn_sensor(world, blueprint_id, attributes, transform, parent):
    blueprint = world.get_blueprint_library().find(blueprint_id)
    for key, value in attributes.items():
        blueprint.set_attribute(key, value)
    return world.spawn_actor(blueprint, transform, attach_to=parent)


def transform_fields(transform):
    loc, rot = transform.location, transform.rotation
    return [loc.x, loc.y, loc.z, rot.roll, rot.pitch, rot.yaw]


def drain_lidar(lidar_queue, vehicle_at_frame, step_at_frame, writer, output, lidar_index):
    """Save every sweep waiting in the queue with the vehicle pose of its frame."""
    while True:
        try:
            data = lidar_queue.get_nowait()
        except queue.Empty:
            return lidar_index
        if data.frame not in vehicle_at_frame:
            continue  # sweep from before recording started
        points = np.frombuffer(data.raw_data, dtype=LIDAR_DTYPE).copy()
        np.save(output / "lidar" / f"{lidar_index:06d}.npy", points)
        writer.writerow([lidar_index, step_at_frame[data.frame], data.frame, f"{data.timestamp:.6f}", len(points)] +
                        [f"{v:.6f}" for v in transform_fields(data.transform)] +
                        [f"{v:.6f}" for v in transform_fields(vehicle_at_frame[data.frame])])
        lidar_index += 1


def main():
    args = parse_args()
    sys.path.insert(0, str(args.agents_path))
    from agents.navigation.basic_agent import BasicAgent  # noqa: E402
    from agents.navigation.global_route_planner import GlobalRoutePlanner  # noqa: E402
    from agents.navigation.local_planner import RoadOption  # noqa: E402

    args.output.mkdir(parents=True, exist_ok=False)
    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()
    cmap = world.get_map()
    assert "Town10HD" in cmap.name, cmap.name
    planner = GlobalRoutePlanner(cmap, 2.0)
    if args.route == "mapping":
        route = mapping_route(cmap, planner, args.mapping_start_road, args.mapping_start_lane)
    else:
        route = outer_ring_route(cmap, args.localization_start_road, args.localization_lane, RoadOption.LANEFOLLOW)
    s, speed, curvature = speed_profile(route, args.max_speed_kph / 3.6, args.lateral_acceleration)
    save_route(args.output / "route.csv", route, s, speed, curvature)
    roads = sorted({wp.road_id for wp, _ in route if not wp.is_junction})
    print(f"route {args.route}: {len(route)} points, {s[-1]:.1f} m, roads {roads}", flush=True)
    metadata = {
        "route": args.route, "routeLengthM": float(s[-1]), "nonJunctionRoads": roads,
        "map": cmap.name, "serverVersion": client.get_server_version(), "clientVersion": client.get_client_version(),
        "fixedDeltaSeconds": FIXED_DELTA_SECONDS, "lidarPeriodSeconds": LIDAR_PERIOD_SECONDS,
        "vehicle": VEHICLE_BLUEPRINT, "lidar": {"blueprint": "sensor.lidar.ray_cast_semantic",
                                                 "attributes": LIDAR_ATTRIBUTES, "mount": LIDAR_MOUNT},
        "imu": {"attributes": IMU_ATTRIBUTES, "mount": {"x": 0.0, "y": 0.0, "z": 0.0}},
        "gnss": {"attributes": GNSS_ATTRIBUTES, "mount": {"x": 0.0, "y": 0.0, "z": 0.0}},
        "maxSpeedKph": args.max_speed_kph, "lateralAccelerationLimit": args.lateral_acceleration,
        "coordinateConvention": "CARLA/Unreal left-handed: x forward, y right, z up; degrees; metres",
        "lidarPointFields": list(LIDAR_DTYPE.names),
        "otherActors": "none spawned; static map props only",
    }
    if args.dry_run:
        (args.output / "metadata.json").write_text(json.dumps(metadata, indent=2))
        return

    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    settings.substepping = True
    settings.max_substep_delta_time = 0.01
    settings.max_substeps = 10
    actors = []
    try:
        world.apply_settings(settings)
        start = route[0][0].transform
        spawn = carla.Transform(carla.Location(start.location.x, start.location.y, start.location.z + 0.5),
                                start.rotation)
        blueprint = world.get_blueprint_library().find(VEHICLE_BLUEPRINT)
        blueprint.set_attribute("role_name", "hero")
        ego = world.spawn_actor(blueprint, spawn)
        actors.append(ego)
        for _ in range(int(2.0 / FIXED_DELTA_SECONDS)):
            ego.apply_control(carla.VehicleControl(hand_brake=True))
            world.tick()
        physics = ego.get_physics_control()
        metadata["vehiclePhysics"] = {"mass": physics.mass,
                                      "centerOfMass": [physics.center_of_mass.x, physics.center_of_mass.y,
                                                       physics.center_of_mass.z],
                                      "boundingBoxCenter": [ego.bounding_box.location.x, ego.bounding_box.location.y,
                                                            ego.bounding_box.location.z],
                                      "boundingBoxExtent": [ego.bounding_box.extent.x, ego.bounding_box.extent.y,
                                                            ego.bounding_box.extent.z]}
        mount = carla.Transform(carla.Location(**LIDAR_MOUNT))
        lidar = spawn_sensor(world, "sensor.lidar.ray_cast_semantic", LIDAR_ATTRIBUTES, mount, ego)
        imu = spawn_sensor(world, "sensor.other.imu", IMU_ATTRIBUTES, carla.Transform(), ego)
        gnss = spawn_sensor(world, "sensor.other.gnss", GNSS_ATTRIBUTES, carla.Transform(), ego)
        collision = spawn_sensor(world, "sensor.other.collision", {}, carla.Transform(), ego)
        actors += [lidar, imu, gnss, collision]
        queues = {name: queue.Queue() for name in ["lidar", "imu", "gnss"]}
        collisions = []
        lidar.listen(queues["lidar"].put)
        imu.listen(queues["imu"].put)
        gnss.listen(queues["gnss"].put)
        collision.listen(lambda event: collisions.append((event.frame, event.other_actor.type_id)))

        agent = BasicAgent(ego, target_speed=args.max_speed_kph,
                           opt_dict={"ignore_traffic_lights": True, "ignore_stop_signs": True,
                                     "ignore_vehicles": True, "sampling_resolution": 2.0},
                           map_inst=cmap, grp_inst=planner)
        agent.set_global_plan(route, stop_waypoint_creation=True, clean_queue=True)
        route_xy = np.array([[wp.transform.location.x, wp.transform.location.y] for wp, _ in route])
        (args.output / "lidar").mkdir()
        tick_file = open(args.output / "ticks.csv", "w", newline="")
        frame_file = open(args.output / "lidar_frames.csv", "w", newline="")
        ticks = csv.writer(tick_file)
        frames = csv.writer(frame_file)
        ticks.writerow(["step", "frame", "time_s", "x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg",
                        "vx", "vy", "vz", "wx_degps", "wy_degps", "wz_degps", "ax", "ay", "az",
                        "throttle", "steer", "brake", "steer_fl_deg", "steer_fr_deg",
                        "imu_ax", "imu_ay", "imu_az", "imu_gx", "imu_gy", "imu_gz", "imu_compass",
                        "gnss_lat", "gnss_lon", "gnss_alt", "route_index", "speed_limit_mps"])
        frames.writerow(["lidar_index", "step", "frame", "time_s", "points",
                         "sensor_x", "sensor_y", "sensor_z", "sensor_roll_deg", "sensor_pitch_deg", "sensor_yaw_deg",
                         "vehicle_x", "vehicle_y", "vehicle_z", "vehicle_roll_deg", "vehicle_pitch_deg",
                         "vehicle_yaw_deg"])
        route_index = 0
        lidar_index = 0
        vehicle_at_frame = {}
        step_at_frame = {}
        step = 0
        stopped_since = None
        wall = time.time()
        released = False
        while True:
            location = ego.get_location()
            window = route_xy[route_index:route_index + 40]
            route_index += int(np.argmin(np.hypot(window[:, 0] - location.x, window[:, 1] - location.y)))
            ahead = min(len(route) - 1, int(np.searchsorted(s, s[route_index] + 6.0)))
            limit = float(min(speed[route_index:ahead + 1].min(), args.max_speed_kph / 3.6))
            agent.set_target_speed(3.6 * max(limit, 2.0))
            control = agent.run_step()
            if not released:
                control.hand_brake = False
                released = True
            ego.apply_control(control)
            frame = world.tick()
            step_at_frame[frame] = step
            snapshot_transform = ego.get_transform()
            velocity = ego.get_velocity()
            angular = ego.get_angular_velocity()
            acceleration = ego.get_acceleration()
            imu_data = queues["imu"].get(timeout=10.0)
            while imu_data.frame < frame:
                imu_data = queues["imu"].get(timeout=10.0)
            assert imu_data.frame == frame, (imu_data.frame, frame)
            gnss_fields = ["", "", ""]
            while not queues["gnss"].empty():
                data = queues["gnss"].get()
                if data.frame == frame:
                    gnss_fields = [f"{data.latitude:.10f}", f"{data.longitude:.10f}", f"{data.altitude:.4f}"]
            # Sweeps arrive shortly after tick() returns and CARLA's float
            # sensor timer gives a period of five or six steps. Each sweep is
            # therefore matched to the vehicle state of its own frame number.
            vehicle_at_frame[frame] = snapshot_transform
            lidar_index = drain_lidar(queues["lidar"], vehicle_at_frame, step_at_frame, frames,
                                      args.output, lidar_index)
            steer_fl = ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
            steer_fr = ego.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
            ticks.writerow([step, frame, f"{imu_data.timestamp:.6f}"] +
                           [f"{v:.6f}" for v in transform_fields(snapshot_transform)] +
                           [f"{v:.6f}" for v in (velocity.x, velocity.y, velocity.z, angular.x, angular.y, angular.z,
                                                 acceleration.x, acceleration.y, acceleration.z)] +
                           [f"{control.throttle:.4f}", f"{control.steer:.4f}", f"{control.brake:.4f}",
                            f"{steer_fl:.4f}", f"{steer_fr:.4f}"] +
                           [f"{v:.6f}" for v in (imu_data.accelerometer.x, imu_data.accelerometer.y,
                                                 imu_data.accelerometer.z, imu_data.gyroscope.x,
                                                 imu_data.gyroscope.y, imu_data.gyroscope.z, imu_data.compass)] +
                           gnss_fields + [route_index, f"{limit:.3f}"])
            step += 1
            speed_now = math.hypot(velocity.x, velocity.y)
            if step % 500 == 0:
                print(f"step {step} t={step * FIXED_DELTA_SECONDS:.1f}s sweeps={lidar_index} "
                      f"route {s[route_index]:.0f}/{s[-1]:.0f} m speed {speed_now:.1f} m/s "
                      f"wall {time.time() - wall:.0f}s", flush=True)
            if agent.done() or (route_index >= len(route) - 2 and speed_now < 0.5) or \
                    s[-1] - s[route_index] < 1.0:
                break
            if speed_now < 0.1 and step * FIXED_DELTA_SECONDS > 5.0:
                stopped_since = stopped_since or step
                if (step - stopped_since) * FIXED_DELTA_SECONDS > 15.0:
                    raise RuntimeError(f"Vehicle stopped at route {s[route_index]:.1f} m")
            else:
                stopped_since = None
            if args.stop_after_seconds is not None and step * FIXED_DELTA_SECONDS >= args.stop_after_seconds:
                break
            if step * FIXED_DELTA_SECONDS > args.max_seconds:
                raise RuntimeError("Drive exceeded --max-seconds")
        deadline = time.time() + 3.0
        while time.time() < deadline:
            time.sleep(0.1)
            lidar_index = drain_lidar(queues["lidar"], vehicle_at_frame, step_at_frame, frames,
                                      args.output, lidar_index)
        tick_file.close()
        frame_file.close()
        metadata.update({"steps": step, "sweeps": lidar_index, "collisions": collisions,
                         "completedRouteM": float(s[route_index]), "wallSeconds": time.time() - wall})
        (args.output / "metadata.json").write_text(json.dumps(metadata, indent=2))
        print(f"done: {step} steps, {lidar_index} sweeps, collisions {len(collisions)}", flush=True)
    finally:
        for actor in actors[1:]:
            try:
                actor.stop()
            except Exception:
                pass
        for actor in reversed(actors):
            try:
                actor.destroy()
            except Exception:
                pass
        world.apply_settings(original)


if __name__ == "__main__":
    main()
