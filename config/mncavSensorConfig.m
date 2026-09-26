function cfg = mncavSensorConfig()
% mncavSensorConfig Source-qualified sensors and declared simulation priors.
% The runtime lateral observer uses the vehicle DBW IMU, not the NovAtel
% reference IMU. Random-walk specifications are not per-sample noise sigma.
% Rate, simulation sigma, empirical correction and physical latency are
% intentionally separate quantities; unspecified hardware remains unknown.
    cfg = jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
        'mncavSensorParameters.json')));
end
