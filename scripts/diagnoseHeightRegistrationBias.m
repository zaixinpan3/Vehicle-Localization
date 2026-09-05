function report = diagnoseHeightRegistrationBias(outputFolder, stage)
% diagnoseHeightRegistrationBias: Controlled same-route D2D failure analysis.
% This is an exploratory diagnostic, not an independent accuracy benchmark.
% Production configuration is unchanged. Query frames never enter future maps.
    arguments
        outputFolder (1,1) string
        stage (1,1) string {mustBeMember(stage,["baseline","mechanisms"])} = "baseline"
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    previous=load(fullfile(root,'output','height_retention','height_map_evaluation.mat'),'maps','frames');
    mapCfg=featureMapBuildConfig(); mapCfg.logEnabled=false;
    pcfg=perceptionConfig(); cloudCfg=pcfg.coarseProbabilityCloud;
    matPath=fullfile(root,'data',mapCfg.pointCloudMatPath);
    posePath=fullfile(root,'data',mapCfg.poseMatchCsvPath);
    frames=previous.frames;
    cachePath=fullfile(outputFolder,'diagnostic_inputs.mat');
    if isfile(cachePath)
        loaded=load(cachePath,'inputs'); inputs=loaded.inputs;
    else
        inputs=cell(numel(frames),1);
        for i=1:numel(frames)
            row=readFramePoseTable(posePath,frames(i));
            [pose,tilt,z]=poseRowToPlanarPose(row);
            frame=loadPointCloudFrame(matPath,frames(i));
            pcfg.coarseProbabilityCloud.projectionRotation=tilt;
            coarse=perceiveCoarseProbabilityCloud(frame,pcfg);
            pcfg.executionMode="offline";
            fine=perceiveFrame(frame,pcfg);
            points=[double(frame.x(:)),double(frame.y(:)),double(frame.z(:))];
            points=points*tilt.';
            queryPoints=cell(3,1);
            for c=1:3
                mask=fine.featureMasks.(cloudCfg.semanticNames(c));
                queryPoints{c}=points(mask(:)&all(isfinite(points),2),:);
            end
            futureFrames=frames(i)+(1:6);
            rows=readFramePoseTable(posePath,futureFrames);
            observations=collectFeatureObservations(matPath,futureFrames,rows,pcfg,mapCfg);
            fineQuery=gridCloud(queryPoints,cloudCfg);
            r=yawRotation(pose(3));
            futurePoints=cell(3,1);
            for c=1:3
                futurePoints{c}=(vertcat(observations.pointsByFeatureFrame{c,:})-[pose(1:2),z])*r;
            end
            futureGrid=transformCloud(gridCloud(futurePoints,cloudCfg),pose,z);
            fixed=temporalMapToProbabilityCloud(previous.maps{i});
            check=registerPointsToGlobalFrame([1 2 3;-20 8 -1],row);
            split=([1 2 3;-20 8 -1]*tilt.')*r.'+[pose(1:2),z];
            inputs{i}=struct('frame',frames(i),'pose',pose,'height',z,'tilt',tilt, ...
                'coarse',coarse,'fine',fineQuery,'fixed',fixed,'futureGrid',futureGrid, ...
                'fineSelf',transformCloud(fineQuery,pose,z), ...
                'coarseSelf',transformCloud(projectSemanticProbabilityCloud(coarse,3),pose,z), ...
                'queryPoints',{queryPoints},'futurePoints',{futurePoints}, ...
                'observations',observations,'coordinateMaximumError',max(abs(check-split),[],'all'));
            fprintf('Cached diagnostic frame %d.\n',frames(i));
        end
        save(cachePath,'inputs','-v7.3');
    end
    if stage=="mechanisms"
        report=mechanismStudy(inputs,outputFolder,matPath,posePath,mapCfg,cloudCfg);
        return;
    end
    resultRows=cell(0,18); classRows=cell(0,11); scanRows=cell(0,6);
    geometryRows=cell(0,10); gradientRows=cell(0,5);
    starts=[0.5 -0.4 deg2rad(2);-0.5 0.4 -deg2rad(2);0 0 0];
    for i=1:numel(inputs)
        item=inputs{i};
        for mapName=["fixed","futureGrid","fineSelf","coarseSelf"]
            for sourceName=["coarse","fine"]
                if mapName=="coarseSelf" && sourceName=="fine", continue; end
                fixed=item.(mapName); moving=item.(sourceName);
                for variant=["xy","xyz","xyzNoCross","xyzExactPose","xyzWideHeight"]
                    if mapName~="fixed" && ~ismember(variant,["xy","xyz","xyzExactPose"]), continue; end
                    cfg=distributionRegistrationConfig(); cfg.method="densityOverlap"; cfg.heightTranslation=item.height;
                    cfg.heightMode="xyz";
                    if variant=="xy", cfg.heightMode="xy"; end
                    f=fixed; m=moving;
                    if variant=="xyzNoCross"
                        f=removeCross(f); m=removeCross(m);
                    elseif variant=="xyzExactPose"
                        cfg.heightStandardDeviation=0; cfg.tiltStandardDeviation=0;
                    elseif variant=="xyzWideHeight"
                        cfg.heightStandardDeviation=1;
                    end
                    baseScore=scoreSemanticProbabilityCloudAlignment(f,m,item.pose,cfg);
                    for startIndex=1:size(starts,1)
                        result=registerSemanticProbabilityCloud(f,m,item.pose+starts(startIndex,:),cfg);
                        delta=result.poseXYTheta-item.pose;
                        bodyDelta=delta(1:2)*yawRotation(item.pose(3),2);
                        resultRows(end+1,:)={item.frame,mapName,sourceName,variant,startIndex, ...
                            result.accepted,result.reason,baseScore,result.similarity, ...
                            norm(delta(1:2)),rad2deg(atan2(sin(delta(3)),cos(delta(3)))), ...
                            bodyDelta(1),bodyDelta(2),min(result.curvatureEigenvalues), ...
                            max(result.curvatureEigenvalues),result.poseXYTheta(1), ...
                            result.poseXYTheta(2),result.poseXYTheta(3)}; %#ok<AGROW>
                        if mapName=="fixed" && startIndex==1 && ismember(variant,["xy","xyz"])
                            for name=cloudCfg.semanticNames
                                fc=selectClass(f,name); mc=selectClass(m,name);
                                [s0,d0]=scoreSemanticProbabilityCloudAlignment(fc,mc,item.pose,cfg);
                                [s1,d1]=scoreSemanticProbabilityCloudAlignment(fc,mc,result.poseXYTheta,cfg);
                                classRows(end+1,:)={item.frame,sourceName,variant,name, ...
                                    fc.components.numComponents,mc.components.numComponents,s0,s1, ...
                                    d0.gradient(1),d0.gradient(2),d1.gradient(3)}; %#ok<AGROW>
                            end
                        end
                    end
                end
            end
        end
        for sourceName=["coarse","fine"]
            for name=["all",cloudCfg.semanticNames]
                f=selectClass(item.fixed,name); m=selectClass(item.(sourceName),name);
                cfg=distributionRegistrationConfig(); cfg.method="densityOverlap"; cfg.heightMode="xyz";
                for dz=-1:0.05:1
                    cfg.heightTranslation=item.height+dz;
                    score=scoreSemanticProbabilityCloudAlignment(f,m,item.pose,cfg);
                    scanRows(end+1,:)={item.frame,sourceName,name,dz,score,"fixedRecordedXYTheta"}; %#ok<AGROW>
                end
            end
        end
        for name=cloudCfg.semanticNames
            for sourceName=["fixed","coarse","fine"]
                c=selectClass(projectSemanticProbabilityCloud(item.(sourceName),3),name);
                c=c.components;
                for j=1:c.numComponents
                    cov=c.covariance(:,:,j); beta=cov(1:2,1:2)\cov(1:2,3);
                    conditionalVariance=cov(3,3)-cov(3,1:2)*beta;
                    geometryRows(end+1,:)={item.frame,sourceName,name,j,c.mean(j,3), ...
                        sqrt(cov(3,3)),sqrt(max(0,conditionalVariance)),norm(beta), ...
                        sqrt(det(cov(1:2,1:2))),c.mixtureWeight(j)}; %#ok<AGROW>
                end
            end
        end
        for mode=["xy","xyz"]
            cfg=distributionRegistrationConfig(); cfg.method="densityOverlap"; cfg.heightMode=mode; cfg.heightTranslation=item.height;
            [f,m]=prepareSemanticRegistration(item.fixed,item.coarse,cfg);
            f.mean(:,1:2)=f.mean(:,1:2)-item.pose(1:2);
            if mode=="xyz", f.mean(:,3)=f.mean(:,3)-item.height; m.mean(:,3)=m.mean(:,3)-item.height; end
            [f,m]=balanceSemanticDistributions(f,m);
            testPose=[0.17 -0.23 item.pose(3)+0.01];
            [~,gradient]=semanticGaussianOverlap(f,m,testPose);
            for axis=1:3
                step=zeros(1,3); step(axis)=1e-5;
                numeric=(semanticGaussianOverlap(f,m,testPose+step)-semanticGaussianOverlap(f,m,testPose-step))/(2e-5);
                gradientRows(end+1,:)={item.frame,mode,axis,gradient(axis),numeric}; %#ok<AGROW>
            end
        end
        fprintf('Completed diagnostic ablations for frame %d.\n',item.frame);
    end
    report=struct();
    report.registration=cell2table(resultRows,'VariableNames',{'frame','map','source','variant','start', ...
        'accepted','reason','recordedPoseScore','optimizedScore','translationDifferenceM','yawDifferenceDeg', ...
        'longitudinalDifferenceM','lateralDifferenceM','minimumCurvature','maximumCurvature','x','y','yaw'});
    report.classes=cell2table(classRows,'VariableNames',{'frame','source','mode','class','mapComponents', ...
        'sourceComponents','recordedPoseScore','optimizedPoseScore','recordedGradientX','recordedGradientY','optimizedGradientYaw'});
    report.heightScan=cell2table(scanRows,'VariableNames',{'frame','source','class','heightOffsetM','score','condition'});
    report.geometry=cell2table(geometryRows,'VariableNames',{'frame','source','class','component','meanZ', ...
        'marginalHeightSdM','conditionalHeightSdM','conditionalSlopeNorm','xyAreaFactor','mixtureWeight'});
    report.gradients=cell2table(gradientRows,'VariableNames',{'frame','mode','axis','analytic','finiteDifference'});
    report.coordinateMaximumError=cellfun(@(x)x.coordinateMaximumError,inputs);
    for field=["registration","classes","heightScan","geometry","gradients"]
        writetable(report.(field),fullfile(outputFolder,field+".csv"));
    end
    save(fullfile(outputFolder,'diagnostic_results.mat'),'report');
    disp(report.registration(report.registration.map=="fixed" & report.registration.start==1,:));
end

function cloud=gridCloud(pointsByClass,cfg)
% Diagnostic point-grid estimator: identical cell size and covariance bounds.
% All cell members contribute; only the supplied fine masks select points.
    cloud=struct('semanticName',strings(0,1),'mean',zeros(0,3), ...
        'covariance',zeros(3,3,0),'mixtureWeight',zeros(0,1),'numComponents',0);
    for c=1:numel(cfg.semanticNames)
        points=pointsByClass{c};
        valid=all(isfinite(points),2) & points(:,1)>=cfg.xMin & points(:,1)<cfg.xMax & ...
            points(:,2)>=cfg.yMin & points(:,2)<cfg.yMax;
        points=points(valid,:);
        if isempty(points), continue; end
        bins=floor((points(:,1:2)-[cfg.xMin,cfg.yMin])/cfg.resolution);
        [~,~,ids]=unique(bins,'rows');
        for j=1:max(ids)
            p=points(ids==j,:); meanP=mean(p,1); dx=p-meanP;
            cov=(dx.'*dx)/size(p,1);
            [v,d]=eig(cov(1:2,1:2)+cfg.regularizationVariance*eye(2),'vector');
            a=v*diag(min(max(d,cfg.minCovarianceEigenvalue),cfg.maxCovarianceEigenvalue))*v.';
            b=cov(1:2,3); zz=max(cov(3,3)+cfg.regularizationVariance, ...
                b.'*(a\b)+cfg.minimumConditionalHeightVariance);
            cloud.semanticName(end+1,1)=cfg.semanticNames(c);
            cloud.mean(end+1,:)=meanP;
            cloud.covariance(:,:,end+1)=[a,b;b.',zz];
            cloud.mixtureWeight(end+1,1)=1-exp(-size(p,1)/cfg.occupancySaturationPointCount);
        end
    end
    cloud.numComponents=numel(cloud.mixtureWeight);
    cloud.mixtureWeight=cloud.mixtureWeight/max(sum(cloud.mixtureWeight),eps);
    cloud=struct('components',cloud);
end

function out=transformCloud(cloud,pose,z)
    cloud=validateSemanticProbabilityCloud(cloud);
    out=cloud; r=yawRotation(pose(3));
    out.mean=cloud.mean*r.'+[pose(1:2),z];
    for j=1:cloud.numComponents, out.covariance(:,:,j)=r*cloud.covariance(:,:,j)*r.'; end
    out=struct('components',out);
end

function r=yawRotation(yaw,dim)
    if nargin<2, dim=3; end
    r=[cos(yaw),-sin(yaw),0;sin(yaw),cos(yaw),0;0 0 1];
    r=r(1:dim,1:dim);
end

function out=removeCross(cloud)
    out=validateSemanticProbabilityCloud(projectSemanticProbabilityCloud(cloud,3));
    out.covariance(1:2,3,:)=0; out.covariance(3,1:2,:)=0;
    out=struct('components',out);
end

function out=selectClass(cloud,name)
    c=validateSemanticProbabilityCloud(cloud);
    % Return a minimal schema while retaining measured height for XY inputs.
    dim=2;
    if size(c.mean,2)==3 || (isfield(c,'heightAvailable') && all(c.heightAvailable)), dim=3; end
    out=validateSemanticProbabilityCloud(projectSemanticProbabilityCloud(cloud,dim));
    keep=out.semanticName==name | name=="all";
    out.semanticName=out.semanticName(keep); out.mean=out.mean(keep,:);
    out.covariance=out.covariance(:,:,keep); out.mixtureWeight=out.mixtureWeight(keep);
    out.mixtureWeight=out.mixtureWeight/max(sum(out.mixtureWeight),eps);
    out.numComponents=nnz(keep);
    out=struct('components',out);
end

function report=mechanismStudy(inputs,folder,matPath,posePath,mapCfg,cloudCfg)
% Fit one diagnostic pitch correction on frame 260 curb pairs only; evaluate
% all other frames/classes without refitting. This is not an extrinsic update.
    beforeRows=heightResiduals(inputs);
    before=cell2table(beforeRows,'VariableNames',{'frame','mapFrame','class','travelM', ...
        'pairs','medianZResidualM','iqrZResidualM'});
    calibration=before(before.frame==260 & before.class=="curb",:);
    angle=atan(calibration.travelM\calibration.medianZResidualM);
    rotation=[cos(angle) 0 -sin(angle);0 1 0;sin(angle) 0 cos(angle)];
    corrected=inputs;
    rows=cell(0,13); classRows=cell(0,9); scanRows=cell(0,7);
    for i=1:numel(inputs)
        item=inputs{i}; corrected{i}=item;
        correctionLocal=item.tilt*rotation*item.tilt.';
        for c=1:3
            corrected{i}.queryPoints{c}=item.queryPoints{c}*correctionLocal.';
        end
        observations=item.observations;
        for j=1:6
            poseRow=observations.framePoseTable(j,:);
            [pose,tilt,z]=poseRowToPlanarPose(poseRow);
            r=yawRotation(pose(3))*tilt;
            globalCorrection=r*rotation*r.';
            for c=1:3
                p=observations.pointsByFeatureFrame{c,j};
                observations.pointsByFeatureFrame{c,j}=(p-[pose(1:2),z])*globalCorrection.'+[pose(1:2),z];
            end
        end
        corrected{i}.observations=observations;
        fixedMap=buildSlidingWindowMap(observations,mapCfg);
        corrected{i}.fixed=temporalMapToProbabilityCloud(fixedMap);
        frame=loadPointCloudFrame(matPath,item.frame);
        pcfg=perceptionConfig(); pcfg.coarseProbabilityCloud.projectionRotation=item.tilt*rotation;
        corrected{i}.coarse=perceiveCoarseProbabilityCloud(frame,pcfg);
        corrected{i}.fine=gridCloud(corrected{i}.queryPoints,cloudCfg);
        p=cell(3,1);
        for c=1:3
            p{c}=(vertcat(observations.pointsByFeatureFrame{c,:})-[item.pose(1:2),item.height])*yawRotation(item.pose(3));
        end
        corrected{i}.futureGrid=transformCloud(gridCloud(p,cloudCfg),item.pose,item.height);
        for geometry=["original","pitchCorrected"]
            current=item;
            if geometry=="pitchCorrected", current=corrected{i}; end
            for mapName=["fixed","futureGrid"]
                for mode=["xy","xyz"]
                    cfg=distributionRegistrationConfig(); cfg.method="densityOverlap"; cfg.heightMode=mode; cfg.heightTranslation=item.height;
                    rows=[rows;registrationRows(current.(mapName),current.coarse,item,cfg,geometry+"_"+mapName,"all")]; %#ok<AGROW>
                end
            end
        end
        % Separate broad support mass from component geometry and EM weights.
        fixed=item.fixed;
        for weightMode=["equalMass","equalPeakXY","emMass"]
            f=fixed;
            if weightMode=="equalMass"
                w=ones(f.components.numComponents,1);
            elseif weightMode=="equalPeakXY"
                w=f.components.supportAmplitude;
            else
                w=zeros(f.components.numComponents,1);
                original=load('output/height_retention/height_map_evaluation.mat','maps');
                layerMap=original.maps{i}.batchMaps(1).gmmMap;
                for layer=layerMap.layers(:).'
                    keep=f.components.semanticName==string(layer.classLabel);
                    w(keep)=layer.componentMixtureWeights(layer.componentSupportAmplitudes>0);
                end
            end
            f.components.mixtureWeight=w/sum(w);
            for mode=["xy","xyz"]
                cfg=distributionRegistrationConfig(); cfg.method="densityOverlap"; cfg.heightMode=mode; cfg.heightTranslation=item.height;
                rows=[rows;registrationRows(f,item.coarse,item,cfg,weightMode,"all")]; %#ok<AGROW>
            end
        end
        % A wide search tests objective bias rather than initial-bound clipping.
        for name=["all",cloudCfg.semanticNames]
            for mode=["xy","xyz"]
                cfg=distributionRegistrationConfig(); cfg.method="densityOverlap"; cfg.heightMode=mode; cfg.heightTranslation=item.height;
                cfg.maximumPoseCorrection=[12 12 deg2rad(12)]; cfg.maximumIterationsPerScale=100;
                cfg.minimumComponents=1;
                f=selectClass(item.fixed,name); m=selectClass(item.coarse,name);
                rows=[rows;registrationRows(f,m,item,cfg,"wideSearch",name)]; %#ok<AGROW>
                for dx=-10:0.1:10
                    pose=item.pose+[dx*cos(item.pose(3)),dx*sin(item.pose(3)),0];
                    score=scoreSemanticProbabilityCloudAlignment(f,m,pose,cfg);
                    scanRows(end+1,:)={item.frame,"original",mode,name,dx,0,score}; %#ok<AGROW>
                end
            end
        end
        fprintf('Pitch/weight/search mechanisms evaluated at frame %d.\n',item.frame);
    end
    % At frame 550, reverse the time side and then extend the future window.
    item=inputs{2}; windowMaps=cell(2,1);
    schedules={544:549,551:580};
    for j=1:2
        frames=schedules{j};
        pcfg=perceptionConfig();
        observations=collectFeatureObservations(matPath,frames,readFramePoseTable(posePath,frames),pcfg,mapCfg);
        windowMaps{j}=buildSlidingWindowMap(observations,mapCfg);
        f=temporalMapToProbabilityCloud(windowMaps{j});
        for mode=["xy","xyz"]
            cfg=distributionRegistrationConfig(); cfg.method="densityOverlap"; cfg.heightMode=mode; cfg.heightTranslation=item.height;
            cfg.maximumPoseCorrection=[12 12 deg2rad(12)]; cfg.maximumIterationsPerScale=100;
            rows=[rows;registrationRows(f,item.coarse,item,cfg,"window"+frames(1)+"to"+frames(end),"all")]; %#ok<AGROW>
        end
    end
    % An unmatched pole class is still given equal nominal class energy.
    % Record each class at both the recorded pose and the returned XY pose.
    for i=1:3
        item=inputs{i}; cfg=distributionRegistrationConfig(); cfg.method="densityOverlap";
        result=registerSemanticProbabilityCloud(item.fixed,item.coarse,item.pose,cfg);
        for name=cloudCfg.semanticNames
            f=selectClass(item.fixed,name); m=selectClass(item.coarse,name);
            [s,d]=scoreSemanticProbabilityCloudAlignment(f,m,result.poseXYTheta,cfg);
            classRows(end+1,:)={item.frame,name,m.components.numComponents,f.components.numComponents, ...
                s,d.gradient(1),d.gradient(2),d.gradient(3),result.accepted}; %#ok<AGROW>
        end
    end
    report=struct(); report.pitchCorrectionDegrees=rad2deg(angle);
    report.rawResiduals=before;
    report.correctedResiduals=cell2table(heightResiduals(corrected),'VariableNames',before.Properties.VariableNames);
    report.mechanisms=cell2table(rows,'VariableNames',{'frame','mechanism','class','mode','start','accepted','reason', ...
        'referenceScore','optimizedScore','translationDifferenceM','yawDifferenceDeg','longitudinalDifferenceM','lateralDifferenceM'});
    report.objectiveScan=cell2table(scanRows,'VariableNames',{'frame','geometry','mode','class', ...
        'longitudinalOffsetM','yawOffsetDeg','score'});
    report.classConflict=cell2table(classRows,'VariableNames',{'frame','class','sourceComponents','mapComponents', ...
        'score','gradientX','gradientY','gradientYaw','fullAccepted'});
    for field=["rawResiduals","correctedResiduals","mechanisms","objectiveScan","classConflict"]
        writetable(report.(field),fullfile(folder,field+".csv"));
    end
    save(fullfile(folder,'mechanism_results.mat'),'report','corrected','windowMaps','angle','rotation','-v7.3');
    fprintf('Diagnostic pitch correction: %.6f degrees.\n',report.pitchCorrectionDegrees);
    disp(report.mechanisms(report.mechanisms.start==1,:));
end

function rows=registrationRows(f,m,item,cfg,mechanism,name)
    rows=cell(3,13); starts=[.5 -.4 deg2rad(2);-.5 .4 -deg2rad(2);0 0 0];
    score=scoreSemanticProbabilityCloudAlignment(f,m,item.pose,cfg);
    for j=1:3
        r=registerSemanticProbabilityCloud(f,m,item.pose+starts(j,:),cfg);
        delta=r.poseXYTheta-item.pose; body=delta(1:2)*yawRotation(item.pose(3),2);
        rows(j,:)={item.frame,mechanism,name,cfg.heightMode,j,r.accepted,r.reason,score,r.similarity, ...
            norm(delta(1:2)),rad2deg(atan2(sin(delta(3)),cos(delta(3)))),body(1),body(2)};
    end
end

function rows=heightResiduals(inputs)
% Nearest XY same-class points within 0.3 m; no GMM and no Z correspondence.
    rows=cell(0,7); names=["curb","roadMarking"];
    for i=1:numel(inputs)
        item=inputs{i}; r=yawRotation(item.pose(3));
        for j=1:6
            poseRow=item.observations.framePoseTable(j,:);
            travel=norm([poseRow.odom_x_m,poseRow.odom_y_m]-item.pose(1:2));
            for c=1:2
                q=item.queryPoints{c};
                p=(item.observations.pointsByFeatureFrame{c,j}-[item.pose(1:2),item.height])*r;
                [distance2,index]=min((q(:,1)-p(:,1).').^2+(q(:,2)-p(:,2).').^2,[],2);
                keep=distance2<0.3^2;
                dz=p(index(keep),3)-q(keep,3);
                rows(end+1,:)={item.frame,item.frame+j,names(c),travel,nnz(keep), ...
                    median(dz),diff(quantile(dz,[.25 .75]))}; %#ok<AGROW>
            end
        end
    end
end
