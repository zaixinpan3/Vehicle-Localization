function data=refineDowntownMappingCurbs(data,perceptionCfg,loadFrame)
% Refine registered curb returns before their export and Gaussian fitting.
% Mapping entry points supply the original frame loader for bend confirmation.
% Registered-only inputs can reproduce the earlier straight-road refinement.
    c=find(string(data.featureNames)=="curb");
    if isempty(c) || ~isfield(perceptionCfg,'downtownCurb'), return; end
    poses=data.framePoseTable;
    if all(ismember({'pose_x_m','pose_y_m'},poses.Properties.VariableNames))
        path=poses{:,{'pose_x_m','pose_y_m'}};
    else
        path=poses{:,{'odom_x_m','odom_y_m'}};
    end
    validate=[];
    if nargin>=3 && ~isempty(loadFrame)
        calibration=data.frameCalibration;
        ids=data.frameIndices;
        validate=@(k,xyz) confirmFrame(xyz,poses(k,:),calibration, ...
            loadFrame(ids(k)),perceptionCfg.downtownCurb.bendStep);
    end
    data.pointsByFeatureFrame(c,:)=refineDowntownCurbRuns( ...
        data.pointsByFeatureFrame(c,:),path,perceptionCfg.downtownCurb.straightRun,validate);
    data.counts(:,c)=cellfun(@(p) size(p,1),data.pointsByFeatureFrame(c,:)).';
    if isfield(data,'frameSummaryTable'), data.frameSummaryTable.curb=data.counts(:,c); end
end

function keep=confirmFrame(xyz,pose,calibration,frame,cfg)
    [rotation,origin]=localizationSupport.poseRowToRigidTransform(pose);
    sensor=((xyz-origin)*rotation-calibration.translation)*calibration.rotation;
    raw=double([frame.x(:),frame.y(:),frame.z(:)]);
    raw=raw(all(isfinite(raw),2) & any(raw~=0,2),:);
    keep=confirmDowntownBendCurbStep(sensor,raw,cfg);
end
