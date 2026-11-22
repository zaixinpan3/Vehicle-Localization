function [rate,valid]=upstreamYawRate(lateral)
% Only a finite rate from the upstream motion packet can enable this linear row.
    n=numel(lateral.time);
    rate=nan(n,1);
    if isfield(lateral,'yawRate')
        validateattributes(lateral.yawRate,{'numeric'},{'real','vector','numel',n});
        rate=double(lateral.yawRate(:));
    end
    valid=isfinite(rate);
    if isfield(lateral,'yawRateValid')
        mask=lateral.yawRateValid;
        assert(numel(mask)==n && all(ismember(mask(:),[0,1])), ...
            'VehicleLocalization:InvalidMotionValidity','Invalid upstream yaw-rate validity mask.');
        valid=valid & logical(mask(:));
    end
    if isfield(lateral,'diagnostics') && isfield(lateral.diagnostics,'dynamicModelEvaluated')
        active=lateral.diagnostics.dynamicModelEvaluated;
        assert(numel(active)==n && all(ismember(active(:),[0,1])), ...
            'VehicleLocalization:InvalidMotionValidity','Invalid lateral model availability.');
        valid=valid & logical(active(:));
    end
end
