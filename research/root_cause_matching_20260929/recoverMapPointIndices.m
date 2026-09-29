function recoverMapPointIndices()
% recoverMapPointIndices Recover preserved raw identities from saved map XYZ.
% This is an offline provenance operation, never online feature classification.
    root=setupVehicleLocalization();out='output/root_cause_matching_20260929';a=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');d=a.featureData;
    mc=featureMapBuildConfig();store=matfile(fullfile(root,'data',mc.pointCloudMatPath));indices=cell(size(d.pointsByFeatureFrame));frames=[];first=0;maximumError=zeros(1170,numel(d.featureNames));
    for k=1:1170
        if isempty(frames)||k>=first+numel(frames),first=k;frames=store.pointClouds(1,k:min(1170,k+49));end
        raw=frames(k-first+1);xyz=double([raw.x(:),raw.y(:),raw.z(:)]);[R,t]=poseRowToRigidTransform(d.framePoseTable(k,:));cal=d.frameCalibration;
        lattice=round(xyz*1e4);
        for c=1:numel(d.featureNames)
            world=d.pointsByFeatureFrame{c,k};if isempty(world),indices{c,k}=zeros(0,1);continue;end
            local=((world-t)*R-cal.translation)*cal.rotation;
            [found,ids]=ismember(round(local*1e4),lattice,'rows');
            for j=find(~found).'
                [distance,ids(j)]=min(sum((xyz-local(j,:)).^2,2));assert(distance<1e-8,'No raw identity for frame %d class %s',k,string(d.featureNames(c)));
            end
            maximumError(k,c)=max(vecnorm(xyz(ids,:)-local,2,2));assert(maximumError(k,c)<1e-4);
            indices{c,k}=ids;
        end
    end
    save(fullfile(out,'map_point_indices.mat'),'indices','maximumError');
    report=struct('frames',1170,'totalPoints',sum(cellfun(@numel,indices),'all'),'maximumReconstructionErrorMeters',max(maximumError,[],'all'),'scope',"offline inverse SE(3) reconstruction, raw-point identity lookup; original map annotations preserved");
    fid=fopen(fullfile(fileparts(mfilename('fullpath')),'map_identity_audit.json'),'w');fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);disp(report);
end
