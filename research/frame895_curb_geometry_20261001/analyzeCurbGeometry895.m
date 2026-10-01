function analyzeCurbGeometry895()
% analyzeCurbGeometry895 Offline point, support-scale and motion interventions.
% Fine labels and reference transport are diagnostic oracles, never runtime inputs.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    out=fullfile(root,'output/frame895_curb_geometry_20261001');if ~isfolder(out),mkdir(out);end
    d=load('output/frame895_diagnosis_20260930/diagnostic.mat');
    a=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');fd=a.featureData;
    a=load('output/root_cause_matching_20260929/finalSurface_sources.mat','currentClouds');cached=a.currentClouds;
    a=load('output/line_direction_matching_20260928/sources.mat','motion');motion=a.motion;
    a=load('output/line_direction_matching_20260928/production/report.mat','report');calls=a.report.calls;
    frames=891:895;refs=cell(5,1);
    for worker=1:4
        a=load(sprintf('output/fine_matching_20260919/inputs_%d.mat',worker),'frames','selectedIndices','cfg');
        for j=1:5
            at=find(a.frames==frames(j));if ~isempty(at),refs{j}=double(a.selectedIndices{at,string(a.cfg.featureNames)=="curb"});end
        end
    end
    cfg=perceptionConfig('Mississippi');cfg.coarseProbabilityCloud.storeDiagnostics=true;mc=featureMapBuildConfig();
    points=cell(5,1);perceptions=cell(5,1);variants=cell(5,3);pillars=cell(0,13);coverage=cell(5,9);
    mapPoints=cell(1170,1);mapClass=fd.featureNames=="curb";
    for k=1:1170
        xy=fd.pointsByFeatureFrame{mapClass,k}(:,1:2);mapPoints{k}=(xy-d.ref(1:2))*rot(d.ref(3));
    end
    for j=1:5
        k=frames(j);raw=loadPointCloudFrame(fullfile(root,'data',mc.pointCloudMatPath),k);
        [~,tilt]=poseRowToPlanarPose(fd.framePoseTable(k,:));cfg.coarseProbabilityCloud.projectionRotation=tilt;
        p=perceiveFrame(raw,cfg);perceptions{j}=p;current=p.probabilityCloud;
        assert(isequaln(current.components,cached{k}.components),'Production coarse cloud changed.');
        xyz=double([raw.x(:),raw.y(:),raw.z(:)]);fine=xyz(refs{j},:);g=p.diagnostics.ground;
        bin=floor((fine(:,1:2)-g.cellOrigin)./g.cellSize)+1;
        valid=all(bin>=1,2)&bin(:,1)<=g.cellMapSize(2)&bin(:,2)<=g.cellMapSize(1);fine=fine(valid,:);bin=bin(valid,:);
        ids=sub2ind(g.cellMapSize,bin(:,2),bin(:,1));selected=find(g.curbCellMask);
        fineMom=aggregatePlanarCellMoments(fine,ids,prod(g.cellMapSize),current.projectionRotation,current.projectionTranslation);
        [boundary,details]=estimateCurbBoundaryGeometry(g,p.diagnostics.groundPointContext,current.projectionRotation,current.projectionTranslation,cfg.curbBoundary);
        fineXY=fine*current.projectionRotation.'+current.projectionTranslation;
        allXY=xyz*current.projectionRotation.'+current.projectionTranslation;
        points{j}=struct('fine',fineXY,'all',allXY,'finePillars',ids,'selected',selected);
        coverage(j,:)={k,numel(refs{j}),numel(ids),nnz(ismember(ids,selected)),numel(selected),nnz(fineMom.count(selected)==0), ...
            nnz(details.accepted),max(abs(current.components.mean-cached{k}.components.mean),[],'all'),cfg.curbBoundary.enabled};
        for id=selected.'
            at=find(details.pillar==id);accepted=false;shift=[0 0];if ~isempty(at),accepted=details.accepted(at);shift=[details.dx(at),details.dy(at)];end
            pillars(end+1,:)={k,id,g.moments.count(id),fineMom.count(id),g.moments.mean(id,1),g.moments.mean(id,2), ...
                boundary.mean(id,1),boundary.mean(id,2),fineMom.mean(id,1),fineMom.mean(id,2),accepted,shift(1),shift(2)}; %#ok<AGROW>
        end
        variants{j,1}=current;
        replacements={g.moments.mean,boundary.mean};has=fineMom.count>0;replacements{2}(has,:)=fineMom.mean(has,:);
        geometry=current.geometry;b= floor((boundary.mean(selected,:)-[geometry.xMin geometry.yMin])/geometry.resolution)+1;
        cellIds=sub2ind(geometry.dims,b(:,2),b(:,1));
        for v=1:2
            c=current;
            for id=find(c.components.semanticName=="curb").'
                local=selected(cellIds==double(c.components.cellLinIdx(id)));assert(~isempty(local));
                w=double(g.moments.count(local));mu=sum(replacements{v}(local,:).*w,1)/sum(w);
                c.components.mean(id,:)=mu;c.components.meanXYZ(id,1:2)=mu;
            end
            variants{j,v+1}=c;
        end
    end
    coverage=cell2table(coverage,VariableNames={'frame','originalFinePoints','finePointsInRoi','coveredFinePoints','selectedPillars','zeroFinePillars','acceptedBoundaryFits','reproductionMaxM','boundaryEnabled'});
    pillars=cell2table(pillars,VariableNames={'frame','pillar','groundPoints','finePoints','wholeX','wholeY','boundaryX','boundaryY','fineX','fineY','boundaryAccepted','proposedDx','proposedDy'});
    writetable(coverage,fullfile(dest,'coverage.csv'));writetable(pillars,fullfile(dest,'pillar_geometry.csv'));
    pools=cell(3,2);matches=cell(3,2);rows=cell(0,7);labels=["production","whole_pillar_centers","fine_pillar_centers"];
    for v=1:3
        for transport=1:2
            history=[];
            for j=1:5
                k=frames(j);pose=motion(k,:);if transport==2,pose=calls{k,{'referenceX','referenceY','referencePsi'}};end
                [cloud,history]=updateLocalizationSourceWindow(variants{j,v},calls.timeSeconds(k),pose,history,d.wc);
            end
            pools{v,transport}=cloud;result=matchLocalProbabilityCloud(d.fixed,cloud,d.initial,d.cfg);matches{v,transport}=result;
            label=labels(v);if transport==2,label=label+"_reference_transport";end
            rows(end+1,:)=metric(label,result,d.ref); %#ok<AGROW>
        end
    end
    assert(isequaln(pools{1,1},d.source));assert(max(abs(matches{1,1}.poseXYTheta-d.results{1}.poseXYTheta))<1e-7);
    controls=cell2table(rows,VariableNames={'variant','errorM','longitudinalM','lateralM','yawErrorDeg','accepted','correspondences'});
    writetable(controls,fullfile(dest,'center_controls.csv'));
    directions=cell(0,8);mapRows=cell(0,5);
    for k=1:1170
        q=mapPoints{k};q=q(q(:,1)>=5&q(:,1)<=13&q(:,2)>-3&q(:,2)<-1,:);
        if size(q,1)>=5 && max(q(:,1))-min(q(:,1))>3
            [angle,scatter]=pcaAngle(q);mapRows(end+1,:)={k,size(q,1),angle,mean(q(:,2)-tand(angle)*(q(:,1)-9)),scatter}; %#ok<AGROW>
        end
    end
    mapPerFrame=cell2table(mapRows,VariableNames={'frame','points','angleDeg','yAt9M','normalStdM'});
    writetable(mapPerFrame,fullfile(dest,'map_observation_directions.csv'));
    for j=1:5
        q=points{j}.fine;pose=calls{frames(j),{'referenceX','referenceY','referencePsi'}};
        q(:,1:2)=(q(:,1:2)*rot(pose(3)).'+pose(1:2)-d.ref(1:2))*rot(d.ref(3));
        for limits=[5 7 9;13 11 17]
            in=q(:,1)>=limits(1)&q(:,1)<=limits(2)&q(:,2)>-3&q(:,2)<-1;
            [angle,scatter]=pcaAngle(q(in,1:2));
            directions(end+1,:)={frames(j),limits(1),limits(2),nnz(in),angle,scatter,min(q(in,1)),max(q(in,1))}; %#ok<AGROW>
        end
    end
    directions=cell2table(directions,VariableNames={'frame','xMin','xMax','points','angleDeg','normalStdM','actualMinX','actualMaxX'});
    writetable(directions,fullfile(dest,'fine_directions.csv'));
    save(fullfile(out,'audit.mat'),'d','fd','calls','motion','points','perceptions','variants','pools','matches','mapPoints','coverage','pillars','controls','directions','mapPerFrame','-v7.3');
    disp(coverage);disp(controls);disp(directions);disp(mapPerFrame);
end
function row=metric(name,r,ref)
    e=r.poseXYTheta-ref;b=e(1:2)*rot(ref(3));
    row={name,norm(e(1:2)),b(1),b(2),rad2deg(atan2(sin(e(3)),cos(e(3)))),r.accepted,height(r.correspondences)};
end
function [angle,scatter]=pcaAngle(q)
    assert(size(q,1)>=3);x=q-mean(q,1);[V,E]=eig(x.'*x/size(x,1),'vector');[~,k]=max(E);t=V(:,k);if t(1)<0,t=-t;end
    angle=atan2d(t(2),t(1));scatter=sqrt(min(E));
end
function r=rot(a),r=[cos(a) -sin(a);sin(a) cos(a)];end
