function data=refineDowntownMappingCurbs(data,perceptionCfg)
% Apply sequence-level fine perception before exporting points or fitting a map.
    c=find(string(data.featureNames)=="curb");
    if isempty(c) || ~isfield(perceptionCfg,'downtownCurb'), return; end
    poses=data.framePoseTable;
    if all(ismember({'pose_x_m','pose_y_m'},poses.Properties.VariableNames))
        path=poses{:,{'pose_x_m','pose_y_m'}};
    else
        path=poses{:,{'odom_x_m','odom_y_m'}};
    end
    data.pointsByFeatureFrame(c,:)=refineDowntownCurbRuns( ...
        data.pointsByFeatureFrame(c,:),path,perceptionCfg.downtownCurb.straightRun);
    data.counts(:,c)=cellfun(@(p) size(p,1),data.pointsByFeatureFrame(c,:)).';
    if isfield(data,'frameSummaryTable'), data.frameSummaryTable.curb=data.counts(:,c); end
end
