function estimate=run(data,cfg,options)
% Validate observer dynamics against a prescribed synthetic quadratic cost.
    arguments
        data (1,1) struct
        cfg (1,1) struct
        options.LateralInputs (1,1) struct=struct()
    end
    if ~isfield(data,'lidar')
        estimate=runFullLocalizationObserver(data,cfg,LateralInputs=options.LateralInputs);
        return
    end
    assert(~isfield(data,'lidarOverlap'),'VehicleLocalization:AmbiguousLidarInput','Supply one test source.');
    source=syntheticSource(data.lidar,data.highRate.time(:));
    lidar=struct('name',"syntheticResidual",'scaleLadder',[],'valid',source.valid, ...
        'frameReliability',source.frameReliability,'directionReliability',source.directionReliability);
    if isfield(source,'evaluateResidual')
        lidar.evaluate=source.evaluateResidual;
    else
        lidar.evaluate=@(k,predicted)syntheticResidual(source,k,predicted);
    end
    estimate=observerInternal.integrate(rmfield(data,'lidar'),cfg,options.LateralInputs,lidar);
end

function measurement=syntheticResidual(source,k,predicted)
    r=predicted-source.values(k,:).';
    r(3)=atan2(sin(r(3)),cos(r(3)));
    measurement=struct('residual',r,'jacobian',eye(3),'weights',source.information(:,:,k),'linearizationPose',predicted);
end

function s=syntheticSource(a,t)
    n=numel(t);
    s=struct('frameReliability',ones(n,1),'directionReliability',ones(n,3));
    assert(isfield(a,'delay') && isequal(a.delay,0),'VehicleLocalization:FullZeroDelayRequired','Declare zero perception delay.');
    assert(isequal(a.time(:),t),'VehicleLocalization:SynchronousInputsRequired','Source clocks must equal the localization clock.');
    assert(isequal(size(a.pose),[n,3]) && numel(a.valid)==n && ...
        all(ismember(a.valid,[0,1])) && size(a.information,1)==3 && size(a.information,2)==3 && size(a.information,3)==n, ...
        'VehicleLocalization:InvalidFullSource','Invalid synchronized source dimensions.');
    s.valid=logical(a.valid(:));s.values=a.pose;s.information=a.information;
    for name=["frameReliability","directionReliability"]
        if isfield(a,name)
            assert(isequal(size(a.(name)),size(s.(name))) && isreal(a.(name)) && ...
                all(isfinite(a.(name)),'all') && all(a.(name)>=0 & a.(name)<=1,'all'), ...
                'VehicleLocalization:InvalidLidarReliability','Invalid aligned reliability array.');
            s.(name)=a.(name);
        end
    end
    if isfield(a,'evaluateResidual')
        assert(isa(a.evaluateResidual,'function_handle'),'VehicleLocalization:InvalidLidarResidual', ...
            'evaluateResidual must be a function handle.');
        s.evaluateResidual=a.evaluateResidual;
    end
    for k=find(s.valid).'
        I=s.information(:,:,k);
        assert(isreal(s.values(k,:)) && all(isfinite(s.values(k,:))) && isreal(I) && all(isfinite(I),'all') && ...
            norm(I-I.','fro')<=1e-10*max(1,norm(I,'fro')), ...
            'VehicleLocalization:InvalidFullSource','Valid frames require finite symmetric information.');
        validSpectrum=min(eig((I+I.')/2))>=-1e-10*max(1,norm(I,2));
        assert(validSpectrum, ...
            'VehicleLocalization:InvalidFullSource','Synthetic information must be positive semidefinite.');
    end
end
