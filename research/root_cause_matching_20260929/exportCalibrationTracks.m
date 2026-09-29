function exportCalibrationTracks()
% exportCalibrationTracks Offline static-pole tracks for timing/origin diagnosis.
% These use existing fine-map annotations and reference poses, never online labels.
    root=setupVehicleLocalization();out='output/root_cause_matching_20260929';dest=fileparts(mfilename('fullpath'));
    a=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');d=a.featureData;timing=load(fullfile(out,'point_timing.mat'));identities=load(fullfile(out,'map_point_indices.mat'),'indices');
    o=load('output/line_direction_matching_20260928/sources.mat','motion');old=load('output/line_direction_matching_20260928/production/report.mat','report');t=old.report.calls.timeSeconds;motion=o.motion;
    at=find(string(d.featureNames)=="pole");rows=cell(0,14);lastXY=zeros(0,2);lastFrame=zeros(0,1);
    for k=1:1170
        p=d.pointsByFeatureFrame{at,k};if size(p,1)<5,continue;end
        pair=(p(:,1)-p(:,1).').^2+(p(:,2)-p(:,2).').^2;groups=conncomp(graph(sparse(pair<.3^2),'omitselfloops'));
        [pose,~,~]=poseRowToPlanarPose(d.framePoseTable(k,:));R=rotation(pose(3));v=[0 0];w=0;
        if k>1,dt=t(k)-t(k-1);v=(motion(k,1:2)-motion(k-1,1:2))*rotation(motion(k-1,3))/dt;w=atan2(sin(motion(k,3)-motion(k-1,3)),cos(motion(k,3)-motion(k-1,3)))/dt;end
        times=timing.pointTimes(:,:,k);pt=double(times(identities.indices{at,k}));used=false(size(lastFrame));
        for id=unique(groups)
            select=groups==id;q=p(select,:);if size(q,1)<8||max(q(:,3))-min(q(:,3))<.8,continue;end
            mu=mean(q(:,1:2),1);distance=vecnorm(lastXY-mu,2,2);distance(used|lastFrame<k-10)=Inf;
            [nearest,track]=min(distance);
            if isempty(nearest)||nearest>1.2,track=size(lastXY,1)+1;lastXY(track,:)=mu;lastFrame(track,1)=k;used(track,1)=false;end
            lastXY(track,:)=mu;lastFrame(track)=k;used(track)=true;
            local=(q(:,1:2)-pose(1:2))*R;dtp=pt(select);angle=w*dtp;
            corrected=[cos(angle).*local(:,1)-sin(angle).*local(:,2),sin(angle).*local(:,1)+cos(angle).*local(:,2)]+dtp.*v;
            meanZero=mean(corrected*R.'+pose(1:2),1);derivative=(v+w*[-mean(local(:,2)),mean(local(:,1))])*R.';
            centered=q-mean(q,1);slope=(centered(:,3).'*centered(:,1:2))/sum(centered(:,3).^2);
            rows(end+1,:)={k,track,size(q,1),pose(3),mu(1),mu(2),meanZero(1),meanZero(2),derivative(1),derivative(2),mean(dtp),slope(1),slope(2),std(q(:,3))}; %#ok<AGROW>
        end
    end
    result=cell2table(rows,VariableNames={'frame','track','points','yaw','originalX','originalY','zeroPhaseX','zeroPhaseY','derivativeX','derivativeY','pointTime','slopeXZ','slopeYZ','stdZ'});
    writetable(result,fullfile(dest,'calibration_tracks.csv'));disp(size(result));
end
function R=rotation(x)
    R=[cos(x) -sin(x);sin(x) cos(x)];
end
