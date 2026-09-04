function rootDir = setupVehicleLocalization()
% setupVehicleLocalization: Add the vehicleLocalization modules to the MATLAB
% path in pipeline order: configuration, perception (with its ground and
% off-ground feature branches and the semantic product), mapping (with the
% temporal-stability Gaussian support map), and localization (with the LPV
% lateral-velocity observer and the seven-state improved observer).
%
% Input:
%   none
%
% Output:
%   rootDir: absolute path of the vehicleLocalization root folder
    rootDir = fileparts(mfilename("fullpath"));
    addpath(fullfile(rootDir, "config"));
    addpath(fullfile(rootDir, "perception"));
    addpath(fullfile(rootDir, "perception", "groundSegmentation"));
    addpath(fullfile(rootDir, "perception", "groundFeatures"));
    addpath(fullfile(rootDir, "perception", "offGroundFeatures"));
    addpath(fullfile(rootDir, "perception", "semanticProduct"));
    addpath(fullfile(rootDir, "mapping"));
    addpath(fullfile(rootDir, "mapping", "temporalStabilityGmm"));
    addpath(fullfile(rootDir, "localization"));
    addpath(fullfile(rootDir, "localization", "lateralObserver"));
    addpath(fullfile(rootDir, "localization", "improvedObserver"));
end
