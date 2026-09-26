function parameters=mncavVehicleConfig()
% mncavVehicleConfig Source-qualified UMN MnCAV nominal vehicle parameters.
% Stock geometry/mass describe the 2021 Pacifica Hybrid, not the loaded test
% vehicle. Inertia and axle stiffness are fitted under those nominal conditions.
% Steering angles supplied to the bicycle model are ROAD-wheel radians;
% subtract config/mncavReplayInterface.json steeringWheelOffsetRad, then
% divide by parameters.steeringRatio. Re-synthesize gains after parameter changes.
    root=fileparts(fileparts(mfilename('fullpath')));
    parameters=jsondecode(fileread(fullfile(root,'config','mncavVehicleParameters.json')));
end
