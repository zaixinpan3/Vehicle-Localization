function validateLandmarkViewConfig(cfg)
% validateLandmarkViewConfig Validate the offline/runtime view-model contract.
    fields=["mergeRadius","maximumAssignmentDistance","minimumObservationPoints", ...
        "bandwidth","coverageScale","varianceFloor","maximumHeadingDifference"];
    valid=isstruct(cfg)&&isscalar(cfg)&&all(isfield(cfg,[fields,"pointClasses"]));
    if valid
        for name=fields
            x=cfg.(name);valid=valid&&isnumeric(x)&&isscalar(x)&&isreal(x)&&isfinite(x)&&x>0;
        end
        valid=valid&&isstring(cfg.pointClasses)&&isrow(cfg.pointClasses)&& ...
            all(ismember(cfg.pointClasses,["pole","trafficSign"]))&&numel(unique(cfg.pointClasses))==numel(cfg.pointClasses) && ...
            cfg.minimumObservationPoints>=3&&cfg.minimumObservationPoints==fix(cfg.minimumObservationPoints) && ...
            cfg.maximumHeadingDifference<=pi;
    end
    assert(valid,'VehicleLocalization:InvalidLandmarkViewConfig','Require valid positive view-map scales and point classes.');
end
