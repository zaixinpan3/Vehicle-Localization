"""Compare native INSPVA position increments with its reported velocity.

Run with uv run --offline --with numpy --with pyproj python <this file>.
The output is evaluation data; it must not enter a production observer.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
from pyproj import Proj, Transformer

ROOT = Path(__file__).resolve().parents[2]
DEST = Path(__file__).resolve().parent
OUT = ROOT / "output/lidar_observer_regression_20260930"
SOURCE = ROOT / "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv"
REFERENCE = ROOT / "output/receiver_synchronized_inputs/native_reference.csv"


def main():
    OUT.mkdir(exist_ok=True, parents=True)
    source = np.genfromtxt(SOURCE, delimiter=",", names=True)
    ref = np.genfromtxt(REFERENCE, delimiter=",", names=True)
    assert len(source) == len(ref)
    t = ref["time"]
    assert np.max(np.abs(np.diff(t) - np.diff(source["gps_seconds"]))) < 1e-8
    factors = Proj("EPSG:32615").get_factors(source["longitude_deg"], source["latitude_deg"])
    gamma = np.deg2rad(factors.meridian_convergence)
    scale = np.asarray(factors.meridional_scale)
    velocity = scale[:, None] * np.column_stack((
        np.cos(gamma)*source["east_velocity_mps"] - np.sin(gamma)*source["north_velocity_mps"],
        np.sin(gamma)*source["east_velocity_mps"] + np.cos(gamma)*source["north_velocity_mps"]))
    xy = np.column_stack((ref["x"], ref["y"]))
    projected = np.column_stack(Transformer.from_crs(4326, 32615, always_xy=True).transform(
        source["longitude_deg"], source["latitude_deg"]))
    assert np.max(np.abs(xy-projected)) < 1e-8
    # Same-message positions and velocities share native GPS epochs. This
    # check needs no ROS-to-receiver motion-clock interpolation or fitted lag.
    dt = np.diff(t)
    residual = np.diff(xy, axis=0) - dt[:, None]*(velocity[:-1]+velocity[1:])/2
    intervals = []
    for begin, end in [(2, 116), (80, 85), (83, 84.6), (84.5, 84.6)]:
        mask = (t[1:] > begin) & (t[1:] <= end)
        intervals.append(dict(begin=begin, end=end, coveredSeconds=float(dt[mask].sum()),
                              positionMinusVelocityIntegralM=residual[mask].sum(axis=0).tolist(),
                              velocityResidualRmseMps=float(np.sqrt(np.mean(np.sum((residual[mask]/dt[mask, None])**2, axis=1))))))
    # Constant timing and rigid-output-point alternatives are diagnostic fits
    # over this recording, not calibrations admitted to production.
    midpoint = (t[:-1]+t[1:])/2
    displacement_velocity = np.diff(xy, axis=0)/dt[:, None]
    mask = (midpoint >= 2.5) & (midpoint <= 115.5)
    lags = np.linspace(-.5, .5, 201)
    lag_scores = []
    for lag in lags:
        shifted = np.column_stack([np.interp(midpoint+lag, t, velocity[:, j]) for j in range(2)])
        lag_scores.append(np.sqrt(np.mean(np.sum((displacement_velocity[mask]-shifted[mask])**2, axis=1))))
    yaw = ref["psi"]
    rotation = np.stack((np.column_stack((np.cos(yaw), -np.sin(yaw))),
                         np.column_stack((np.sin(yaw), np.cos(yaw)))), axis=1)
    rotation_delta = np.diff(rotation, axis=0)
    lever = np.linalg.lstsq(rotation_delta[mask].reshape(-1, 2), residual[mask].reshape(-1), rcond=None)[0]
    lever_residual = residual - np.einsum("nij,j->ni", rotation_delta, lever)
    best = int(np.argmin(lag_scores))
    summary = dict(nativeSamples=len(t), projectionScaleRange=[float(scale.min()), float(scale.max())],
                   positionProjectionMaximumDifferenceM=float(np.max(np.abs(xy-projected))), intervals=intervals,
                   diagnosticLag=dict(bestSeconds=float(lags[best]), bestVelocityRmseMps=float(lag_scores[best]),
                                      zeroLagVelocityRmseMps=float(lag_scores[100]), searchRangeSeconds=[-.5, .5]),
                   diagnosticLeverArm=dict(bodyOffsetM=lever.tolist(),
                      beforeVelocityRmseMps=float(np.sqrt(np.mean(np.sum((residual[mask]/dt[mask,None])**2,axis=1)))),
                      afterVelocityRmseMps=float(np.sqrt(np.mean(np.sum((lever_residual[mask]/dt[mask,None])**2,axis=1))))),
                   limitations="Same receiver diagnostic consistency, not an independent truth assessment. Fits do not identify physical latency or mounting geometry. No production inputs or calibration changed.")
    np.savetxt(OUT/"native_grid_velocity.csv", np.column_stack((t, velocity, scale)), delimiter=",",
               header="time,vx,vy,projectionScale", comments="")
    (DEST/"reference_consistency.json").write_text(json.dumps(summary, indent=2)+"\n")
    inputs = [SOURCE, REFERENCE, ROOT/"output/support_full_localization_20260930/observer/experiment.mat"]
    hashes = {str(path.relative_to(ROOT)): hashlib.file_digest(path.open("rb"), "sha256").hexdigest() for path in inputs}
    (DEST/"input_hashes.json").write_text(json.dumps(hashes, indent=2)+"\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
