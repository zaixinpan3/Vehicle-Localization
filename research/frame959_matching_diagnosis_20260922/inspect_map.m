function inspect_map(probabilityCloudMap)
% inspect_map Preserve target provenance and height-control qualifications.
    dest=fileparts(mfilename('fullpath'));out=fullfile(pwd,'output/frame959_matching_diagnosis_20260922');
    s=load(fullfile(out,'diagnostic.mat'));loaded=load('output/mississippi_mapping_calibrated/probability_cloud.mat','cloud');
    map=probabilityCloudMap.canonicalMap;c=loaded.cloud.components;
    ids=unique(s.pairs.globalTarget(s.pairs.semanticName~="curb"));rows=cell(numel(ids),8);support=cell(numel(ids),1);
    for k=1:numel(ids)
        id=ids(k);label=c.semanticName(id);layer=map.layers(map.classLabels==label);
        component=layer.components(layer.componentIds==erase(c.componentId(id),label+":"));
        tile=layer.tiles(all(vertcat(layer.tiles.ownerTile)==component.ownerTile,2));
        frames=str2double(tile.observationBlockIds)+1;counts=component.blockEffectiveCounts;use=counts>.5;
        rows(k,:)={id,c.componentId(id),c.repeatability(id),c.mixtureWeight(id),c.meanXYZ(id,3), ...
            min(frames(use)),max(frames(use)),nnz(use)};
        support{k}=table(repmat(id,numel(frames),1),frames,counts,VariableNames={'globalTarget','frame','effectiveCount'});
    end
    targets=cell2table(rows,VariableNames={'globalTarget','componentId','repeatability','mixtureWeight','meanMapZ', ...
        'firstSupportedFrame','lastSupportedFrame','supportingFramesOverHalfPoint'});
    writetable(targets,fullfile(dest,'map_targets.csv'));writetable(vertcat(support{:}),fullfile(dest,'map_frame_support.csv'));disp(targets);
    % Diagnostic height compatibility requires a known vertical reference and
    % excludes the one local map component with unavailable height statistics.
    keep=vecnorm(c.mean-s.seed(1:2),2,2)<=100;excluded=nnz(keep&~c.heightAvailable);keep=keep&c.heightAvailable;
    fixed=struct('components',struct('semanticName',c.semanticName(keep),'mean',c.meanXYZ(keep,:), ...
        'covariance',c.covarianceXYZ(:,:,keep),'mixtureWeight',c.mixtureWeight(keep),'numComponents',nnz(keep)), ...
        'frameCalibration',loaded.cloud.frameCalibration);
    cfg=featureMapBuildConfig();row=readFramePoseTable(fullfile(pwd,'data',cfg.poseMatchCsvPath),959);
    [~,~,z]=poseRowToPlanarPose(row);reg=s.cfg.registration;reg.heightMode="xyz";reg.heightTranslation=z;
    result=registerSemanticProbabilityCloud(fixed,s.current,s.seed,reg);
    heightControl=struct('accepted',result.accepted,'reason',result.reason,'errorM',norm(result.poseXYTheta(1:2)-s.reference(1:2)), ...
        'rank',result.observableRank,'referenceHeightUsed',true,'excludedMapComponentsWithoutHeight',excluded, ...
        'sourceFrames',1,'productionHeightUsed',false,'limitation',"Diagnostic only; a directional pose is not a validated full-pose localization");
    fid=fopen(fullfile(dest,'height_control.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(heightControl,PrettyPrint=true));
end
