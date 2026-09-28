function captureCurbJointMoments()
% captureCurbJointMoments Capture unlabeled whole-pillar geometry on raw frames.
    setupVehicleLocalization();out='output/frame827_perception_20260928';
    cfg=perceptionConfig('Mississippi');cfg.featureNames="curb";cfg.semanticPrecision.enabled=false;cfg.coarseProbabilityCloud.storeDiagnostics=true;
    store=matfile('data/raw/MissisipiPointClouds.mat');tables=cell(1170,1);started=tic;
    for first=1:50:1170
        raw=store.pointClouds(1,first:min(first+49,1170));
        for j=1:numel(raw)
            frame=first+j-1;p=perceiveFrame(raw(j),cfg);g=p.diagnostics.ground;
            [x,names,ids]=measureCurbJointMoments(g,p.diagnostics.groundPointContext);
            [r,c]=ind2sub(g.cellMapSize,ids);xy=g.cellOrigin+([c r]-.5).*g.cellSize;
            geom=p.candidates.geometry;bins=floor((xy-geom.origin)./geom.cellSize)+1;ids=sub2ind(geom.mapSize,bins(:,2),bins(:,1));
            tables{frame}=[table(repmat(frame,numel(ids),1),ids,VariableNames={'frame','pillar'}),array2table(x,VariableNames=names)];
        end
        fprintf('Joint capture %d/1170 %.1fs\n',first+numel(raw)-1,toc(started));
    end
    writetable(vertcat(tables{:}),fullfile(out,'joint_features.csv'));fprintf('JOINT_CAPTURE_COMPLETED\n');
end
