function runFineControl()
% runFineControl Offline source-representation diagnostic on selected frames.
% Reuse the map-building fine masks, undo their known registration, and bin
% their points at the same 0.9 m resolution. This is not an online alternative
% or an independent accuracy test: the same observations helped build the map.
    setupVehicleLocalization(); maxNumCompThreads(8);
    s=load('output/matching_refinement_20260919/inputs.mat');
    f=load('output/mississippi_mapping_inspva_20260915/feature_observations.mat');
    a=load('output/matching_refinement_20260919/validated/recursive/report.mat');
    c=a.report.calls; ref=c{:,{'referenceX','referenceY','referencePsi'}};
    motion=a.report.deadReckoning{:,{'x','y','psi'}};
    map=registrationSupport.projectSemanticProbabilityCloud(s.map,2);
    selected=unique([30 75 80:100 200 425 600 800:842 900 959 966 1100]);
    rows=cell(0,10); snapshots=cell(0,1);
    for k=selected
        fh=[]; ch=[];
        for j=max(1,k-2):k
            fine=makeFineCloud(f.featureData,j,ref(j,:),s.clouds{j});
            [fine,fh]=updateLocalizationSourceWindow(fine,c.timeSeconds(j),motion(j,:),fh);
            [coarse,ch]=updateLocalizationSourceWindow(s.clouds{j},c.timeSeconds(j),motion(j,:),ch);
        end
        for name=["curb","pole","trafficSign","all"]
            moving=coarse;
            if name=="all"
                moving=fine;
            else
                keep=moving.components.semanticName~=name; add=fine.components.semanticName==name;
                for field=["mean","semanticName","mixtureWeight","semanticProbability","occupancyProbability"]
                    moving.components.(field)=[moving.components.(field)(keep,:);fine.components.(field)(add,:)];
                end
                moving.components.covariance=cat(3,moving.components.covariance(:,:,keep),fine.components.covariance(:,:,add));
                moving.components.numComponents=nnz(keep)+nnz(add);
            end
            initial=c{k,{'predictedX','predictedY','predictedPsi'}};
            r=registerSemanticProbabilityCloud(map,moving,initial,a.cfg.registration);
            e=r.poseXYTheta-ref(k,:); e(3)=atan2(sin(e(3)),cos(e(3)));
            rows(end+1,:)={k,name,norm(e(1:2)),rad2deg(e(3)),r.accepted,r.reason,r.observableRank,r.similarity,moving.components.numComponents,c.positionErrorM(k)}; %#ok<AGROW>
        end
        if ismember(k,[92 820 830])
            snapshots{end+1}=struct('frame',k,'fine',fine,'coarse',coarse,'reference',ref(k,:)); %#ok<AGROW>
        end
    end
    results=cell2table(rows,VariableNames={'frame','replacedClass','candidateErrorM','yawErrorDeg','accepted','reason','rank','similarity','components','productionErrorM'});
    out='output/error_attribution_20260919';
    writetable(results,fullfile(out,'fine_controls.csv'));
    save(fullfile(out,'fine_controls.mat'),'results','snapshots','-v7.3');
end

function cloud=makeFineCloud(f,k,pose,template)
    rot=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
    g=template.geometry; cfg=coarseSemanticProbabilityCloudConfig();
    means=zeros(0,2); covariance=zeros(2,2,0); names=strings(0,1); counts=zeros(0,1);
    for j=1:numel(f.featureNames)
        p=(f.pointsByFeatureFrame{j,k}(:,1:2)-pose(1:2))*rot;
        cellXY=floor((p-[g.xMin,g.yMin])/g.resolution);
        keep=all(cellXY>=0,2) & cellXY(:,1)<g.dims(2) & cellXY(:,2)<g.dims(1);
        p=p(keep,:); cellXY=cellXY(keep,:);
        [~,~,id]=unique(cellXY,'rows');
        for cellId=1:max(id,[],'all')
            points=p(id==cellId,:); mu=mean(points,1); centered=points-mu;
            covar=centered.'*centered/size(points,1)+cfg.regularizationVariance*eye(2);
            [v,d]=eig(covar,'vector'); d=min(cfg.maxCovarianceEigenvalue,max(cfg.minCovarianceEigenvalue,d));
            means(end+1,:)=mu; covariance(:,:,end+1)=v*diag(d)*v.'; %#ok<AGROW>
            names(end+1,1)=f.featureNames(j); counts(end+1,1)=size(points,1); %#ok<AGROW>
        end
    end
    quality=1-exp(-counts/cfg.occupancySaturationPointCount);
    cloud=struct('dimension',2,'frameCalibration',template.frameCalibration, ...
        'components',struct('mean',means,'covariance',covariance,'semanticName',names, ...
        'numComponents',size(means,1),'mixtureWeight',quality/max(sum(quality),eps), ...
        'semanticProbability',ones(size(counts)),'occupancyProbability',quality));
end
