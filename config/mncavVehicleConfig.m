function parameters=mncavVehicleConfig()
% mncavVehicleConfig Source-qualified UMN MnCAV nominal vehicle parameters.
% Stock geometry/mass describe the 2021 Pacifica Hybrid, not the loaded test
% vehicle. Inertia and axle cornering stiffness remain unmeasured priors.
% Steering angles supplied to the bicycle model are ROAD-wheel radians;
% divide recorded steering-wheel radians by parameters.steeringRatio.
    root=fileparts(fileparts(mfilename('fullpath')));
    parameters=jsondecode(fileread(fullfile(root,'config','mncavVehicleParameters.json')));
end
