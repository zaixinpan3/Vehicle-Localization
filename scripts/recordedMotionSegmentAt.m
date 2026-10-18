function segment=recordedMotionSegmentAt(time,nativeSegment,query)
% recordedMotionSegmentAt Admit interpolation only within one valid segment.
% Exact sensor epochs need one valid sample; interpolated poses need both
% brackets in the same positive segment. Never interpolate across withdrawal.
    time=double(time(:));nativeSegment=double(nativeSegment(:));query=double(query(:));
    assert(numel(time)>=2 && all(diff(time)>0) && numel(nativeSegment)==numel(time), ...
        'VehicleLocalization:InvalidMotionTime','Require aligned segment IDs on an increasing clock.');
    left=interp1(time,nativeSegment,query,'previous');
    right=interp1(time,nativeSegment,query,'next');
    segment=left;segment(~isfinite(left) | left~=right | left<=0)=0;
end
