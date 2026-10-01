function auditFallbackAndMap895()
% auditFallbackAndMap895 Trace rejected boundary geometry and map covariance.
% Outside-pillar centers are diagnostic interventions, not production policy.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/frame895_curb_geometry_20261001';
    a=load(fullfile(out,'audit.mat'));d=a.d;cfg=perceptionConfig('Mississippi');
    names=["production","frame895_outside_boundary","window_outside_boundaries","frame895_drop_bad_pillar"];
    results=cell(4,2);rows=cell(0,7);rejected=cell(0,9);
    for v=1:4
        clouds=a.variants(:,1);
        for j=1:5
            if v==1||(v~=3&&j~=5),continue;end
            p=a.perceptions{j};g=p.diagnostics.ground;c=clouds{j};
            [boundary,t]=estimateCurbBoundaryGeometry(g,p.diagnostics.groundPointContext,c.projectionRotation,c.projectionTranslation,cfg.curbBoundary);
            % Freeze the investigated frame/pillar for the single-defect controls.
            if v==3,keep=~t.accepted&t.explainedFraction>=cfg.curbBoundary.minimumExplainedFraction&t.residualStd<=cfg.curbBoundary.maximumResidualStd;
            else,keep=t.pillar==5947;end
            changed=t.pillar(keep);
            for k=find(keep).'
                id=t.pillar(k);rejected(end+1,:)={names(v),890+j,id,t.dx(k),t.dy(k),t.explainedFraction(k),t.residualStd(k),g.moments.mean(id,1),g.moments.mean(id,2)}; %#ok<AGROW>
            end
            if v==4
                g.curbCellMask(changed)=false;g.moments=boundary;cc=cfg.coarseProbabilityCloud;cc.semanticNames=cfg.featureNames;
                cc.projectionRotation=c.projectionRotation;cc.projectionTranslation=c.projectionTranslation;cc.frameCalibration=c.frameCalibration;
                cloud=buildCoarseSemanticProbabilityCloud(g,p.diagnostics.offGround,cc);cloud=applyCurbBoundaryScatter(cloud,cfg.curbBoundary);
                clouds{j}=cloud;continue;
            end
            replacement=boundary.mean;replacement(changed,:)=g.moments.mean(changed,:)+[t.dx(keep),t.dy(keep)];
            selected=find(g.curbCellMask);geometry=c.geometry;
            bins=floor((boundary.mean(selected,:)-[geometry.xMin geometry.yMin])/geometry.resolution)+1;
            cells=sub2ind(geometry.dims,bins(:,2),bins(:,1));
            for id=find(c.components.semanticName=="curb").'
                local=selected(cells==double(c.components.cellLinIdx(id)));w=double(g.moments.count(local));
                mu=sum(replacement(local,:).*w,1)/sum(w);c.components.mean(id,:)=mu;c.components.meanXYZ(id,1:2)=mu;
            end
            clouds{j}=c;
        end
        for mode=1:2
            history=[];
            for j=1:5
                k=j+890;pose=a.motion(k,:);if mode==2,pose=a.calls{k,{'referenceX','referenceY','referencePsi'}};end
                [cloud,history]=updateLocalizationSourceWindow(clouds{j},a.calls.timeSeconds(k),pose,history,d.wc);
            end
            r=matchLocalProbabilityCloud(d.fixed,cloud,d.initial,d.cfg);results{v,mode}=r;e=r.poseXYTheta-d.ref;b=e(1:2)*rot(d.ref(3));
            rows(end+1,:)={names(v),mode,norm(b),b(1),b(2),rad2deg(atan2(sin(e(3)),cos(e(3)))),r.accepted}; %#ok<AGROW>
        end
    end
    controls=cell2table(rows,VariableNames={'variant','transportVariant','errorM','longitudinalM','lateralM','yawErrorDeg','accepted'});
    rejected=cell2table(rejected,VariableNames={'variant','frame','pillar','dx','dy','explainedFraction','residualStd','wholeX','wholeY'});
    writetable(controls,fullfile(dest,'fallback_controls.csv'));writetable(rejected,fullfile(dest,'fallback_geometry.csv'));
    base=load('output/mississippi_mapping_calibrated/probability_cloud.mat');views=load(featureMapBuildConfig().probabilityCloudPath);
    baseId=views.cloud.mapConstruction.originalComponentGroups{685};assert(isscalar(baseId));componentId=erase(base.cloud.components.componentId(baseId),'curb:');
    original=load('output/mississippi_mapping_calibrated/probability_cloud_map.mat');layers=original.probabilityCloudMap.canonicalMap.layers;
    layer=layers(string({layers.classLabel})=="curb");index=find(layer.componentIds==componentId);component=layer.components(index);assert(isscalar(component));
    covarianceRows=cell(0,7);fields=["covariance","withinBlockCovariance","stableOffsetCovariance","referenceMeanCovariance"];
    for name=fields
        S=rot(d.ref(3)).'*component.(name)*rot(d.ref(3));[angle,minor,major]=axesOf(S);
        covarianceRows(end+1,:)={name,angle,minor,major,S(1,1),S(1,2),S(2,2)}; %#ok<AGROW>
    end
    covariance=cell2table(covarianceRows,VariableNames={'term','angleDeg','minorVariance','majorVariance','xx','xy','yy'});
    writetable(covariance,fullfile(dest,'map_covariance.csv'));
    q=zeros(0,2);acquisition=zeros(0,1);within=zeros(2);used=[];
    for k=1:1170
        p=a.mapPoints{k};p=p(p(:,1)>=5&p(:,1)<=13&p(:,2)>-3&p(:,2)<-1,:);if isempty(p),continue;end
        centered=p-mean(p,1);within=within+centered.'*centered;q=[q;p];acquisition=[acquisition;repmat(k,size(p,1),1)];used(end+1)=k; %#ok<AGROW>
    end
    within=within/size(q,1);centered=q-mean(q,1);total=centered.'*centered/size(q,1);between=total-within;
    decomposition=struct('domain','All original map curb points at reference-body X in [5,13], Y in [-3,-1]; not GMM responsibility assignments', ...
        'frames',used,'points',size(q,1),'within',within,'between',between,'total',total, ...
        'withinAngleDeg',axesOf(within),'totalAngleDeg',axesOf(total),'componentId',componentId, ...
        'curbViewConditioned',any(views.cloud.landmarkViews.config.pointClasses=="curb"));
    fid=fopen(fullfile(dest,'map_moment_decomposition.json'),'w');fprintf(fid,'%s\n',jsonencode(decomposition,PrettyPrint=true));fclose(fid);
    writetable(table(acquisition,q(:,1),q(:,2),VariableNames={'frame','x','y'}),fullfile(out,'local_map_points.csv'));
    p=a.points{5}.all;p=p(p(:,1)>=5&p(:,1)<=17&p(:,2)>=-3&p(:,2)<=-1,:);
    writematrix(p,fullfile(out,'frame895_local_raw.csv'));writematrix(a.points{5}.fine,fullfile(out,'frame895_fine_points.csv'));
    save(fullfile(out,'fallback_map.mat'),'controls','results','rejected','component','covariance','decomposition');
    disp(controls);disp(rejected);disp(covariance);disp(decomposition);
end
function [angle,minor,major]=axesOf(S)
    [v,e]=eig(S,'vector');[major,k]=max(e);minor=min(e);axis=v(:,k);if axis(1)<0,axis=-axis;end;angle=atan2d(axis(2),axis(1));
end
function r=rot(a),r=[cos(a) -sin(a);sin(a) cos(a)];end
