#!/usr/bin/env python3
"""Reproduce the historical interface audit using archived input exports.

uv run --offline --with numpy --with scipy --with pandas python research/mncav_interface_audit_20260916/audit_historical_interfaces.py
This reproduces earlier evidence only; it is not a current Vx input method.
Calibration fits use the separate 12:11:24 drive; all other fits are labeled
diagnostic. No inferred installation geometry is deployed.
"""
from pathlib import Path
import json
import numpy as np
import pandas as pd
from scipy.optimize import least_squares, lsq_linear
from scipy.signal import savgol_filter

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "output/mncav_interface_audit_20260916"
BASE = ROOT / "output/mississippi_20240607_120931_20260907"


def stats(x):
    x = np.asarray(x)
    return {"n": int(len(x)), "mean": float(np.mean(x)), "median": float(np.median(x)),
            "rmse": float(np.sqrt(np.mean(x*x))), "p95Abs": float(np.percentile(np.abs(x), 95))}


def load(seq):
    folder = BASE / ("sensors" if seq == "12-09-31" else "calibration_sensors")
    if seq == "12-09-31":
        ins = pd.read_csv(ROOT / "data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv")
        ins.columns = [c.replace("_deg", "").replace("_mps", "") for c in ins.columns]
    else:
        ins = pd.read_csv(folder / "inspva.csv")
    t = ins.gps_seconds.to_numpy()-ins.gps_seconds.iloc[0]
    ros = ins.stamp_sec.to_numpy()-ins.stamp_sec.iloc[0]
    dt = np.median(np.diff(t)); assert np.max(np.abs(np.diff(t)-dt)) < 1e-6
    derive = lambda x, order=1: savgol_filter(np.asarray(x), 51, 3, deriv=order, delta=dt)
    az = np.deg2rad(ins.azimuth.to_numpy()); yaw = np.unwrap(np.pi/2-az)
    vx = ins.east_velocity.to_numpy()*np.cos(yaw)+ins.north_velocity.to_numpy()*np.sin(yaw)
    vy = -ins.east_velocity.to_numpy()*np.sin(yaw)+ins.north_velocity.to_numpy()*np.cos(yaw)
    r = derive(yaw); rdot = derive(yaw, 2)
    ax, ay = derive(vx)-vy*r, derive(vy)+vx*r
    frame = pd.DataFrame(dict(time=t, rosTime=ros, referenceVx=vx, referenceVy=vy, referenceR=r,
        referenceRDot=rdot, referenceAx=ax, referenceAy=ay, azimuth=ins.azimuth, roll=ins.roll, pitch=ins.pitch))
    sample = lambda tab, field, clock="stamp_sec": np.interp(ins.stamp_sec, tab[clock], tab[field])
    imu, steer, twist = [pd.read_csv(folder / (name+".csv")) for name in ["imu", "steering", "twist"]]
    correction = json.loads((BASE / "vehicle_parameters.json").read_text())["input_correction"]
    for name, key in [("longitudinalAcceleration", "acceleration_x_mps2"), ("lateralAcceleration", "acceleration_y_mps2"), ("yawRate", "angular_z_radps")]:
        frame[name] = correction[name]["sign"]*sample(imu, key)+correction[name]["offset"]
    frame["measuredVx"] = sample(twist, "linear_x_mps")
    frame["steeringWheel"] = sample(steer, "steering_wheel_angle_rad")
    curvature = pd.read_csv(OUT / seq / "steering_curvature.csv")
    # The curvature topic has no header. Pairing by bag time avoids mixing
    # receiver time with wall time; the observed publication gap is audited.
    paired_wheel = np.interp(curvature.bag_time_sec, steer.bag_time_sec, steer.steering_wheel_angle_rad)
    curvature_receiver = np.interp(curvature.bag_time_sec-ins.stamp_sec.iloc[0], ros, t)
    frame["curvature"] = sample(curvature, "data", "bag_time_sec")
    roll = np.deg2rad(ins.roll); pitch = -np.deg2rad(ins.pitch)
    frame["referenceFull3dVy"] = np.cos(roll)*vy+np.sin(roll)*np.sin(pitch)*vx-np.sin(roll)*np.cos(pitch)*ins.up_velocity
    frame["insGood"] = ins.ins_status.to_numpy() == 3 if "ins_status" in ins else True
    metadata = {"samples": len(ins), "receiverDurationSeconds": float(t[-1]), "rosDurationSeconds": float(ros[-1]),
        "tfTopicPresent": any(x["topic"] in ["/tf", "/tf_static"] for x in json.loads((OUT / seq / "manifest.json").read_text())["inventory"]),
        "axisFrameLabels": {"imu": sorted(imu.frame_id.unique().tolist()), "twist": sorted(twist.frame_id.unique().tolist())},
        "steeringCalibrationFaultCount": int(steer.calibration_fault.sum()),
        "steeringCurvaturePairTimeDifferenceSeconds": stats(curvature.bag_time_sec.to_numpy()-steer.bag_time_sec.to_numpy()),
        "qualityFiltering": "INS good flag available" if "ins_status" in ins else "INS status not included in this older export"}
    heading = pd.read_csv(OUT / seq / "heading2.csv")
    valid = (heading.sol_status_status == 0) & (heading.heading_stdev > 0)
    metadata["heading2"] = {"samples": len(heading), "reportedHeadingStdDegrees": stats(heading.heading_stdev[valid]),
        "validWithStdBelowOneDegree": int((valid & (heading.heading_stdev < 1)).sum()),
        "note": "Base-to-rover vector is not a verified vehicle heading; length=-1 alone is not an invalidity test."}
    return frame, metadata, (paired_wheel, curvature.data.to_numpy(), curvature_receiver)


def model_fit(frame, mask, vehicle, offset):
    # Joint force/yaw fit with positive, bounded parameters. The two equations
    # are scaled to acceleration units using nominal mass and wheelbase.
    vx = frame.measuredVx.to_numpy(); vy = frame.referenceVy.to_numpy(); r = frame.referenceR.to_numpy()
    delta = (frame.steeringWheel.to_numpy()-offset)/16.2
    cf, cr, iz = (vehicle[k] for k in ["frontCorneringStiffness", "rearCorneringStiffness", "yawInertia"])
    m, lf, lr = (vehicle[k] for k in ["mass", "lf", "lr"]); length = lf+lr
    af = delta-(vy+lf*r)/np.maximum(vx, 1); ar = -(vy-lr*r)/np.maximum(vx, 1)
    force = np.column_stack((cf*af/m, cr*ar/m, np.zeros(len(vx))))
    moment = np.column_stack((lf*cf*af/(m*length), -lr*cr*ar/(m*length), -iz*frame.referenceRDot/(m*length)))
    matrix = np.vstack((force[mask], moment[mask])); target = np.r_[frame.referenceAy[mask], np.zeros(mask.sum())]
    fit = lsq_linear(matrix, target, bounds=(.3, 3))
    def residual(other_mask):
        f = force[other_mask]@fit.x-frame.referenceAy[other_mask]
        q = moment[other_mask]@fit.x
        return {"forceAccelerationResidual": stats(f), "yawMomentScaledResidual": stats(q)}
    return {"multipliersCfCrIz": fit.x.tolist(), "conditionNumber": float(np.linalg.cond(matrix)),
            "anyBoundReached": bool(np.any((fit.x < .301)|(fit.x > 2.999))), "training": residual(mask),
            "later": residual((frame.time >= 40)&(frame.measuredVx > 5)&(frame.time < frame.time.iloc[-1]-.52)),
            "deployed": False, "interpretation": "Mapping assumed, not measured; coefficients at bounds are not accepted physical parameters."}


def main():
    vehicle = json.loads((ROOT / "config/mncavVehicleParameters.json").read_text())["vehicle"]
    loaded = {seq: load(seq) for seq in ["12-09-31", "12-11-24"]}
    wheel, curve, time = loaded["12-11-24"][2]
    train = (time > 1)&(time < 40)
    length = vehicle["lf"]+vehicle["lr"]
    fit = least_squares(lambda x: (np.tan((wheel[train]-x[0])/16.2)/length-curve[train])*100, [.026])
    offset = float(fit.x[0])
    report = {"confirmedVehicle": "UMN MnCAV, user confirmation", "steeringCalibration": {
        "trainingDrive": "12-11-24", "trainingIntervalSeconds": [1, 40], "wheelbaseM": length, "steeringRatio": 16.2,
        "fittedWheelOffsetRad": offset, "fittedWheelOffsetDeg": float(np.rad2deg(offset)),
        "selectedWheelOffsetRad": float(np.deg2rad(1.5)), "selection": "Round recorded curvature equivalence to 1.5 deg; not an independently measured road-wheel zero."}, "drives": {}}
    for seq, (frame, meta, paired) in loaded.items():
        wheel, curve, time = paired
        old = np.tan(wheel/16.2)/length-curve
        fixed = np.tan((wheel-np.deg2rad(1.5))/16.2)/length-curve
        meta["curvatureConsistency"] = {"before": stats(old), "after": stats(fixed),
            "laterAfter40": stats(fixed[time >= 40])}
        moving = (frame.measuredVx >= 5)&(frame.time > .52)&(frame.time < frame.time.iloc[-1]-.52)
        straight = moving & (np.abs(frame.yawRate) < .03)
        meta["signalConsistency"] = {}
        for measured, ref in [("measuredVx", "referenceVx"), ("longitudinalAcceleration", "referenceAx"), ("lateralAcceleration", "referenceAy"), ("yawRate", "referenceR")]:
            meta["signalConsistency"][measured] = {"movingResidual": stats((frame[measured]-frame[ref])[moving]),
                "straightResidual": stats((frame[measured]-frame[ref])[straight])}
        meta["bodyVelocityProjection"] = {"planarStraight": stats(frame.referenceVy[straight]),
            "full3dStraight": stats(frame.referenceFull3dVy[straight]),
            "interpretation": "Full rotation uses existing roll/pitch convention; no unknown mounting or CG translation added."}
        very_straight = moving & (np.abs(frame.referenceR) < .005)
        meta["bodyVelocityProjection"]["planarAbsYawRateBelow005"] = stats(frame.referenceVy[very_straight])
        ulc = pd.read_csv(OUT / seq / "ulc_report.csv")
        # Compare by absolute header time, not the receiver-time axis.
        folder = BASE / ("sensors" if seq == "12-09-31" else "calibration_sensors")
        twist = pd.read_csv(folder / "twist.csv")
        can_speed = np.interp(ulc.stamp_sec, twist.stamp_sec, twist.linear_x_mps)
        meta["ulcVersusTwistSpeed"] = stats((ulc.speed_meas.to_numpy()-can_speed)[can_speed > 5])
        meta["offsetCommandTopics"] = {}
        for topic in ["offset_angle_error", "pre_offset_angle_error", "pre_pre_offset_angle_error"]:
            command = pd.read_csv(OUT / seq / (topic+".csv"))
            meta["offsetCommandTopics"][topic] = {"samples": len(command),
                "enabledSamples": int(command.control_enabled.sum()), "commandMin": float(command.cmd.min()),
                "commandMax": float(command.cmd.max()), "usedForCalibration": False}
        lags = np.arange(-.20, .201, .01); safe = moving & (frame.time > 1)&(frame.time < frame.time.iloc[-1]-1)
        scores = []
        for lag in lags:
            residual = frame.yawRate.to_numpy()-np.interp(frame.time+lag, frame.time, frame.referenceR)
            scores.append(np.sqrt(np.mean(residual[safe]**2)))
        k = int(np.argmin(scores)); meta["yawTiming"] = {"diagnosticBestShiftSeconds": float(lags[k]),
            "bestRmseRadps": float(scores[k]), "zeroShiftRmseRadps": float(scores[20]),
            "applied": False, "note": "Timing/model/derivative effects confounded; no clock correction selected."}
        selected = moving & (frame.time < 40)&(np.abs(frame.referenceR) > .02)
        meta["boundedPhysicalParameterDiagnostic"] = model_fit(frame, selected, vehicle, np.deg2rad(1.5))
        # A fixed mapping is assessed across drives, not fit on the evaluation
        # drive then presented as independent extrinsic calibration.
        design = np.column_stack((frame.measuredVx, frame.referenceR))
        coeff = np.linalg.lstsq(design[moving], frame.referenceVy[moving], rcond=None)[0]
        meta["descriptiveKinematicFit"] = {"model": "referenceVy = delta*measuredVx + pointX*yawRate",
            "deltaDeg": float(np.rad2deg(coeff[0])), "pointXM": float(coeff[1]),
            "residual": stats((frame.referenceVy-design@coeff)[moving]), "physicalExtrinsicsIdentified": False}
        report["drives"][seq] = meta
        frame.to_csv(OUT / seq / "aligned_interfaces.csv", index=False)
    config = json.loads((OUT / "12-09-31/insconfig.json").read_text())[0]
    report["receiverConfiguration"] = {k: config[k] for k in ["imu_type", "mapping", "profile", "number_of_translations", "translations", "number_of_rotations", "rotations"]}
    report["receiverConfiguration"]["scope"] = "One snapshot near evaluation-drive end; no TF and no measured CG extrinsics. Empty lists do not establish physical coincidence or configuration at every instant."
    c = report["drives"]["12-11-24"]["descriptiveKinematicFit"]
    f = loaded["12-09-31"][0]; mask = (f.measuredVx >= 5)&(f.time > .52)&(f.time < f.time.iloc[-1]-.52)
    report["kinematicFitTransfer"] = {"calibrationDrive": "12-11-24", "evaluationDrive": "12-09-31",
        "residual": stats((f.referenceVy-np.deg2rad(c["deltaDeg"])*f.measuredVx-c["pointXM"]*f.referenceR)[mask]),
        "physicalInterpretation": "Descriptive fit conflates sideslip, mounting, reference point and attitude error. Failure to transfer does not identify a physical extrinsic."}
    (OUT / "interface_audit.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
