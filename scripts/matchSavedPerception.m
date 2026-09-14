function report = matchSavedPerception(observationPath,mapPath,outputDirectory,frameIndices)
% matchSavedPerception Evaluate D2D pose events from cached fine feature points.
% Saved global points are inverted into a gravity-aligned local XY frame using
% their recorded pose. This is an in-sample consistency experiment when these
% frames also contributed to the fixed map, not independent localization truth.
% No perception, map fitting, or registration parameter tuning is performed.
    if nargin<4, frameIndices=[28 214 458 600 855 943 1137]; end
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    s=load(observationPath,'featureData'); data=s.featureData;
    s=load(mapPath,'cloud'); fixed=s.cloud;
    cfg=distributionRegistrationConfig(); pcfg=coarseSemanticProbabilityCloudConfig();
    offsets=[.5 -.4 deg2rad(2);-.5 .4 -deg2rad(2);0 0 0];
    records=cell(numel(frameIndices)*size(offsets,1),1); runs=records;
    sourceClouds=cell(numel(frameIndices),1); index=0;
    for k=1:numel(frameIndices)
        fi=find(data.frameIndices==frameIndices(k)); assert(isscalar(fi),'Missing/duplicate frame.');
        row=data.framePoseTable(fi,:); reference=poseRowToPlanarPose(row);
        [moving,roundTripError]=savedCloud(data,fi,reference,pcfg);
        sourceClouds{k}=moving;
        for start=1:size(offsets,1)
            initial=reference+offsets(start,:); timer=tic;
            result=registerSemanticProbabilityCloud(fixed,moving,initial,cfg); elapsed=toc(timer);
            event=registrationSupport.registrationPoseMeasurement(result,double(row.lidar_stamp_sec));
            delta=result.poseXYTheta-reference; delta(3)=atan2(sin(delta(3)),cos(delta(3)));
            matrix=[]; measurementType="rejected"; minEigenvalue=NaN;
            if ~isempty(event)
                matrix=event.information; measurementType=event.measurementType;
                assert(norm(matrix-matrix.','fro')<1e-9*max(1,norm(matrix,'fro')));
                minEigenvalue=min(eig((matrix+matrix.')/2));
                assert(minEigenvalue>=-1e-9*max(1,norm(matrix,2)));
                if result.accepted, [~,flag]=chol(matrix); assert(flag==0); end
            end
            index=index+1;
            records{index}=struct('frame',frameIndices(k),'start',start,'sourceComponents',moving.components.numComponents, ...
                'initialPose',initial,'referencePose',reference,'pose',result.poseXYTheta, ...
                'measurementType',measurementType,'reason',result.reason,'observableRank',result.observableRank, ...
                'similarity',result.similarity,'xyDifference',norm(delta(1:2)),'yawDifferenceDegrees',rad2deg(delta(3)), ...
                'registrationSeconds',elapsed,'iterations',result.iterations,'converged',result.converged, ...
                'information',matrix,'minimumInformationEigenvalue',minEigenvalue,'roundTripError',roundTripError);
            runs{index}=struct('result',result,'measurement',event);
            fprintf('frame=%d start=%d %s rank=%d XY=%.4f m yaw=%.4f deg\n',frameIndices(k),start,result.reason,result.observableRank,norm(delta(1:2)),rad2deg(delta(3)));
        end
    end
    cases=vertcat(records{:}); types=string({cases.measurementType});
    report=struct('inputObservations',string(observationPath),'inputMap',string(mapPath), ...
        'sourceRepresentation',"cachedFinePointsAggregatedIntoXYGaussians", ...
        'evaluation',"inSamplePosePerturbationConsistency",'frameIndices',frameIndices, ...
        'fullPoseCount',nnz(types=="fullPose"),'directionalPoseCount',nnz(types=="directionalPose"), ...
        'rejectedCount',nnz(types=="rejected"),'informationCoordinates',"additive map X,Y,psi; meters,radians", ...
        'informationCalibrated',false,'cases',cases);
    save(fullfile(outputDirectory,'matching_results.mat'),'report','runs','sourceClouds','cfg','pcfg','offsets','-v7.3');
    fid=fopen(fullfile(outputDirectory,'matching_results.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
end

function [cloud,roundTripError]=savedCloud(data,index,pose,cfg)
% Use the same output resolution and covariance safeguards as the coarse
% cloud, but label this adapter explicitly as a fine-point input product.
    r=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
    means=zeros(0,2);covariances=zeros(2,2,0);names=strings(0,1);counts=zeros(0,1);roundTripError=0;
    for j=1:numel(data.featureNames)
        globalPoints=double(data.pointsByFeatureFrame{j,index}(:,1:2));
        p=(globalPoints-pose(1:2))*r;
        if isempty(p), continue; end
        roundTripError=max(roundTripError,max(abs(p*r.'+pose(1:2)-globalPoints),[],'all'));
        valid=all(isfinite(p),2)&p(:,1)>=cfg.xMin&p(:,1)<cfg.xMax&p(:,2)>=cfg.yMin&p(:,2)<cfg.yMax;
        p=p(valid,:);if isempty(p),continue;end
        [~,~,group]=unique(floor((p-[cfg.xMin cfg.yMin])/cfg.resolution),'rows');
        for g=1:max(group)
            points=p(group==g,:); mu=mean(points,1);centered=points-mu;
            c=centered.'*centered/size(points,1)+cfg.regularizationVariance*eye(2);
            [v,e]=eig((c+c.')/2,'vector');e=max(cfg.minCovarianceEigenvalue,min(cfg.maxCovarianceEigenvalue,e));
            c=v*diag(e)*v.';c=(c+c.')/2;
            means(end+1,:)=mu;covariances(:,:,end+1)=c;names(end+1,1)=string(data.featureNames(j));counts(end+1,1)=size(points,1); %#ok<AGROW>
        end
    end
    assert(roundTripError<1e-8,'Global-to-local reconstruction failed.');
    n=numel(counts);assert(n>0);
    components=struct('semanticName',names,'mean',means,'covariance',covariances, ...
        'mixtureWeight',ones(n,1)/n,'numComponents',n,'semanticProbability',ones(n,1), ...
        'occupancyProbability',min(1,counts/cfg.occupancySaturationPointCount));
    cloud=struct('components',components,'dimension',2,'coordinateFrame',"gravityAlignedLocalXY", ...
        'frameCalibration',data.frameCalibration,'sourceRepresentation',"cachedFinePoints",'sourcePointCounts',counts);
    mappingSupport.validateSemanticProbabilityCloud(cloud);
end
