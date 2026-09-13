function [data,metadata] = reconstructContinuousObserverSignals(high,poseTime,pose,information,cfg,options)
% reconstructContinuousObserverSignals Declare an offline linear reconstruction.
% poseTime is the physical measurement time, never a delivery clock. For LiDAR
% the returned output at t reconstructs the pose at t-fixedLidarDelay. Samples
% with missing directions, large gaps or nonfinite values are rejected. This
% adapter does not establish that recorded sensors delivered continuous outputs.
    arguments
        high (1,1) struct
        poseTime (:,1) double {mustBeFinite}
        pose double {mustBeFinite}
        information double
        cfg (1,1) struct
        options.MaximumGap (1,1) double {mustBePositive,mustBeFinite} = .12
    end
    assert(numel(poseTime)>=2 && all(diff(poseTime)>0), ...
        'VehicleLocalization:InvalidReconstruction','Physical pose times must strictly increase.');
    gap=max(diff(poseTime));
    assert(gap<=options.MaximumGap+1e-12,'VehicleLocalization:ReconstructionGap', ...
        'The recorded pose gap %.6g s exceeds the declared reconstruction limit %.6g s.',gap,options.MaximumGap);
    width=2;delay=0;
    if cfg.mode=="lidar",width=3;delay=cfg.measurement.fixedLidarDelay;end
    assert(isreal(pose) && isequal(size(pose),[numel(poseTime),width]), ...
        'VehicleLocalization:InvalidReconstruction','Unexpected recorded pose shape.');
    time=high.time(:);keep=time>=poseTime(1)+delay & time<=poseTime(end)+delay;
    assert(nnz(keep)>=2,'VehicleLocalization:InvalidReconstruction','No covered reconstruction interval.');
    for name=string(fieldnames(high)).'
        assert(numel(high.(name))==numel(time),'VehicleLocalization:InvalidReconstruction','High-rate inputs must be aligned.');
        high.(name)=high.(name)(keep);high.(name)=high.(name)(:);
    end
    query=high.time-delay;
    source=struct('representation',"piecewiseLinear",'time',high.time);
    if cfg.mode=="gnss"
        source.position=interp1(poseTime,pose,query,'linear');
    else
        assert(isequal(size(information),[3,3,numel(poseTime)]), ...
            'VehicleLocalization:InvalidReconstruction','Each LiDAR pose needs physical information.');
        for k=1:numel(poseTime)
            [~,~,info]=computeLidarInformationWeights(information(:,:,k),cfg);
            assert(info.qualified,'VehicleLocalization:InsufficientLidarInformation', ...
                'Recorded LiDAR cannot be reconstructed through an insufficient direction.');
        end
        pose(:,3)=unwrap(pose(:,3));source.pose=interp1(poseTime,pose,query,'linear');
        source.information=zeros(3,3,numel(query));
        for row=1:3
            for column=1:3
                source.information(row,column,:)=reshape(interp1(poseTime,reshape(information(row,column,:),[],1),query,'linear'),1,1,[]);
            end
        end
        source.delay=delay;source.headingConvention="unwrapped";
    end
    data=struct('highRate',high);data.(cfg.mode)=source;
    metadata=struct('representation',"offline piecewise-linear reconstruction", ...
        'maximumOriginalPoseGapSeconds',gap,'declaredMaximumGapSeconds',options.MaximumGap, ...
        'fixedDelaySeconds',delay,'trimmedInputSamples',nnz(~keep), ...
        'physicalContinuityVerified',false,'onlineCausalityClaimed',false, ...
        'headingUnwrapAssumption',"Adjacent recorded yaw changes are less than pi; cycle slips are not verified.");
end
