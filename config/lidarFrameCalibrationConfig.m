function cfg = lidarFrameCalibrationConfig(profile)
% lidarFrameCalibrationConfig: Stored points to the configured pose reference.
% Apply p_reference = rotation*p_stored + translation before recorded attitude.
% Identity is the default for generic/synthetic inputs. Mississippi selects
% its explicit empirical profile, shared by offline and online callers.
% Carla selects the roof LiDAR of the CARLA Town10HD captures, expressed at
% the vehicle's kinematic reference point (scripts/carla/prepareCarlaDataset.py).
    if nargin<1,profile="identity";end
    profile=lower(string(profile));
    assert(isscalar(profile) && ismember(profile,["identity","mississippi","missisipi","downtown","carla"]), ...
        'VehicleLocalization:UnknownCalibrationProfile','Unknown LiDAR calibration profile.');
    cfg=struct('rotation',eye(3),'translation',[0 0 0],'identifier',"identity");
    if ismember(profile,["mississippi","missisipi"])
        record=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')),'mississippiLidarFrameCalibration.json')));
        assert(record.schemaVersion==1 && string(record.targetFrame)=="recorded_INS_output_point", ...
            'VehicleLocalization:InvalidCalibration','Unexpected calibration target or schema.');
        cfg=validateLidarFrameCalibration(record.calibration);
    elseif profile=="carla"
        record=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')),'carlaTown10LidarFrameCalibration.json')));
        assert(record.schemaVersion==1 && string(record.targetFrame)=="vehicle_kinematic_reference_point", ...
            'VehicleLocalization:InvalidCalibration','Unexpected calibration target or schema.');
        cfg=validateLidarFrameCalibration(record.calibration);
    end
end
