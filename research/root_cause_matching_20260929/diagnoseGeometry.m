function diagnoseGeometry()
% diagnoseGeometry Separate tracking, source geometry and map representation.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    b=load('output/pole_boundary_recovery_20260929/replay.mat','currentClouds','sources');
    p=load('output/line_direction_matching_20260928/production/report.mat','report');calls=p.report.calls;
    o=load('output/line_direction_matching_20260928/sources.mat','motion');mc=featureMapBuildConfig();map=load(mc.probabilityCloudPath,'cloud');
    R=@(x)[cos(x) -sin(x);sin(x) cos(x)];
    for frame=[178 806 854 893]
        ref=calls{frame,{'referenceX','referenceY','referencePsi'}};current=b.currentClouds{frame}.components;f=map.cloud.components;
        fixedMean=(f.mean-ref(1:2))*R(ref(3));rows=cell(0,8);
        for k=max(1,frame-4):frame
            c=b.currentClouds{k}.components;delta=o.motion(k,:)-o.motion(frame,:);
            xy=c.mean*R(delta(3)).'+delta(1:2)*R(o.motion(frame,3));
            for id=find(ismember(c.semanticName,["pole","trafficSign"])).'
                targets=find(f.semanticName==c.semanticName(id));[distance,j]=min(vecnorm(fixedMean(targets,:)-xy(id,:),2,2));target=targets(j);
                rows(end+1,:)={k,c.semanticName(id),xy(id,1),xy(id,2),c.semanticProbability(id),distance,fixedMean(target,1),fixedMean(target,2)}; %#ok<AGROW>
            end
        end
        t=cell2table(rows,VariableNames={'frame','class','x','y','probability','nearestMapDistance','mapX','mapY'});
        writetable(t,fullfile(dest,"history_"+frame+".csv"));disp(frame);disp(t);
        frameData=struct('frame',frame,'current',current,'source',b.sources{frame},'reference',ref,'map',map.cloud,'fixedBodyMean',fixedMean);
        save(fullfile(root,'output/root_cause_matching_20260929',"frame"+frame+".mat"),'frameData');
    end
end
