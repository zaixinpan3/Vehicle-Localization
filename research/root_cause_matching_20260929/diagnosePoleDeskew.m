function diagnosePoleDeskew()
% diagnosePoleDeskew Diagnostic only: reference-twist compensation of scan timing.
% Unknown header-to-scan phase is screened explicitly, never silently calibrated.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/root_cause_matching_20260929';
    timing=load(fullfile(out,'point_timing.mat'));a=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');d=a.featureData;
    refs=cell(1170,1);for worker=1:4
        f=load(sprintf('output/fine_matching_20260919/inputs_%d.mat',worker),'frames','selectedIndices','cfg');refs(f.frames)=f.selectedIndices(:,string(f.cfg.featureNames)=="pole");
    end
    poses=zeros(1170,3);for k=1:1170,poses(k,:)=poseRowToPlanarPose(d.framePoseTable(k,:));end
    t=d.framePoseTable.receiver_time_sec;velocity=gradient(poses(:,1),t);north=gradient(poses(:,2),t);yawRate=gradient(unwrap(poses(:,3)),t);
    f806=load(fullfile(out,'frame806.mat'),'frameData');reference=f806.frameData.reference;R=rotation(reference(3));at=find(string(d.featureNames)=="pole");rows=cell(0,11);
    for k=1:1170
        xyz=d.pointsByFeatureFrame{at,k};xy=(xyz(:,1:2)-reference(1:2))*R;keep=sum((xy-[2.525 -4.394]).^2,2)<=1.3^2;
        if nnz(keep)<3,continue;end
        assert(numel(refs{k})==size(xyz,1));times=timing.pointTimes(:,:,k);pt=double(times(refs{k}(keep)));
        local=(xyz(keep,1:2)-poses(k,1:2))*rotation(poses(k,3));v=[velocity(k) north(k)]*rotation(poses(k,3));
        for phase=[0 .05 .1]
            dt=pt-phase;angle=yawRate(k)*dt;ca=cos(angle);sa=sin(angle);
            moved=[ca.*local(:,1)-sa.*local(:,2),sa.*local(:,1)+ca.*local(:,2)]+dt.*v;
            corrected=(moved*rotation(poses(k,3)).'+poses(k,1:2)-reference(1:2))*R;
            rows(end+1,:)={k,phase,numel(pt),mean(xy(keep,1)),mean(xy(keep,2)),mean(corrected(:,1)),mean(corrected(:,2)),mean(pt),v(1),v(2),yawRate(k)}; %#ok<AGROW>
        end
    end
    q=cell2table(rows,VariableNames={'frame','phase','points','beforeX','beforeY','afterX','afterY','pointTime','vx','vy','yawRate'});writetable(q,fullfile(dest,'pole_deskew_diagnosis_806.csv'));disp(q);
end
function R=rotation(x)
    R=[cos(x) -sin(x);sin(x) cos(x)];
end
