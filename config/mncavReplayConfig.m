function parameters = mncavReplayConfig(calibrationFile)
% mncavReplayConfig Current vehicle/steering plus fixed recorded IMU corrections.
% An optional historical export supplies input_correction only. Its vehicle
% and steering snapshots never override the canonical configuration.
    arguments
        calibrationFile (1,1) string = ""
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    parameters=mncavVehicleConfig();
    corrections=jsondecode(fileread(fullfile(root,'config','mncavInputCorrections.json')));
    parameters.input_correction=corrections.input_correction;
    parameters.inputCorrectionSource=string(fullfile(root,'config','mncavInputCorrections.json'));
    if strlength(calibrationFile)>0
        archived=jsondecode(fileread(calibrationFile));
        if isfield(archived,'input_correction')
            parameters.input_correction=archived.input_correction;
            parameters.inputCorrectionSource=calibrationFile;
        end
    end
    interface=jsondecode(fileread(fullfile(root,'config','mncavReplayInterface.json')));
    parameters.steeringWheelOffsetRad=interface.steeringWheelOffsetRad;
    parameters.steeringInterfaceCalibration=interface;
    parameters.vehicleParameterSource=string(fullfile(root,'config','mncavVehicleParameters.json'));
end
