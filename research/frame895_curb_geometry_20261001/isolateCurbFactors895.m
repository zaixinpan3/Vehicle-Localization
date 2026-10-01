function isolateCurbFactors895()
% isolateCurbFactors895 Keep association, covariance and class weights fixed.
% These unconstrained offline optimizations are not production solver outputs.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    a=load('output/frame895_curb_geometry_20261001/audit.mat');d=a.d;
    map=load('output/frame895_curb_geometry_20261001/fallback_map.mat','component');
    cfg=d.cfg;if ~d.results{1}.pyramid.softFineAssociation,cfg=rmfield(cfg,'softPointAssociation');end
    model=prepareSemanticRegistrationGeometry(d.local,d.source,d.initial,cfg);
    pose=d.results{1}.poseXYTheta;pose(1:2)=pose(1:2)-d.initial(1:2);
    system=model.linearize(pose,ones(3,1));target=d.globalIds(system.pairs.target);selected=target==685;
    options=optimset('Display','off','MaxIter',4000,'MaxFunEvals',16000,'TolX',1e-10,'TolFun',1e-12);
    names=["frozen_production","remove_685_angle_only","remove_685_position_only","remove_685_all", ...
        "fine_current_source_directions","local_map_target_directions","same_frame_map_target_directions", ...
        "within_block_target_angle"];
    rows=cell(numel(names),8);solutions=zeros(numel(names),3);
    supportRows=cell(0,8);mapAll=vertcat(a.mapPoints{:});fine=a.points{5}.fine(:,1:2);
    for k=1:numel(names)
        s=system;mask=ones(3,s.numPairs);
        switch names(k)
            case "remove_685_angle_only",mask(3,selected)=0;
            case "remove_685_position_only",mask(1:2,selected)=0;
            case "remove_685_all",mask(:,selected)=0;
            case "within_block_target_angle"
                [vectors,values]=eig(map.component.withinBlockCovariance,'vector');[~,index]=max(values);axis=vectors(:,index).';
                s.targetNormal(selected,:)=repmat([-axis(2),axis(1)],nnz(selected),1);
            case {"fine_current_source_directions","local_map_target_directions","same_frame_map_target_directions"}
                for j=find(selected).'
                    id=s.pairs.source(j);x=d.source.components.mean(id,1);
                    if names(k)=="fine_current_source_directions"
                        q=fine;
                    elseif names(k)=="local_map_target_directions"
                        q=mapAll;
                    else
                        q=a.mapPoints{895};
                    end
                    q=q(abs(q(:,1)-x)<=4&q(:,2)>-3&q(:,2)<-1,:);
                    if size(q,1)<3||max(q(:,1))-min(q(:,1))<2.4,continue;end
                    [axis,angle,scatter]=direction(q);
                    if names(k)=="fine_current_source_directions",s.sourceTangent(j,:)=axis;
                    else,s.targetNormal(j,:)=[-axis(2),axis(1)]*rot(d.ref(3)).';end
                    supportRows(end+1,:)={names(k),id,x,size(q,1),angle,scatter,min(q(:,1)),max(q(:,1))}; %#ok<AGROW>
                end
        end
        cost=@(p)objective(p,s,model.moving,cfg,mask);
        [p,value,flag]=fminsearch(cost,pose,options);solutions(k,:)=p;
        if k==1,assert(abs(cost(pose)-system.cost)<1e-10);end
        estimate=p+[d.initial(1:2) 0];e=estimate-d.ref;b=e(1:2)*rot(d.ref(3));
        rows(k,:)={names(k),norm(b),b(1),b(2),rad2deg(atan2(sin(e(3)),cos(e(3)))),value,flag,norm(p-pose)};
    end
    controls=cell2table(rows,VariableNames={'variant','errorM','longitudinalM','lateralM','yawErrorDeg','modifiedCost','exitFlag','poseChange'});
    writetable(controls,fullfile(dest,'fixed_factor_controls.csv'));
    support=cell2table(supportRows,VariableNames={'variant','source','centerX','points','angleDeg','normalStdM','actualMinX','actualMaxX'});
    writetable(support,fullfile(dest,'fixed_factor_support.csv'));
    directionRows=cell(0,7);
    for v=1:3
        for t=1:2
            m=prepareSemanticRegistrationGeometry(d.local,a.pools{v,t},d.initial,cfg);c=a.pools{v,t}.components;
            for j=find(c.semanticName=="curb" & c.mean(:,2)<0).'
                axis=m.moving.supportTangent(j,:);if axis(1)<0,axis=-axis;end
                directionRows(end+1,:)={v,t,j,c.mean(j,1),c.mean(j,2),atan2d(axis(2),axis(1)),m.moving.axisConfidence(j)}; %#ok<AGROW>
            end
        end
    end
    directions=cell2table(directionRows,VariableNames={'centerVariant','transportVariant','source','x','y','angleDeg','axisConfidence'});
    writetable(directions,fullfile(dest,'source_directions.csv'));
    save('output/frame895_curb_geometry_20261001/factors.mat','controls','support','directions','system','target','solutions');
    disp(controls);disp(support);
end
function cost=objective(p,s,m,cfg,mask)
    residual=supportRegistrationResiduals(m.mean(s.pairs.source,1:2),m.planarCovariance(:,:,s.pairs.source), ...
        s.targetMean,s.targetCovariance,s.sourceTangent,s.targetNormal,s.directionScale,p,s.noiseStandardDeviation);
    q=sum((residual.*mask).^2,1).';c=cfg.geometric.robustStandardizedDistance^2;
    cost=sum(s.weights.*c.*log1p(q/c));
end
function [axis,angle,scatter]=direction(q)
    q=q-mean(q,1);[v,e]=eig(q.'*q/size(q,1),'vector');[~,k]=max(e);axis=v(:,k).';if axis(1)<0,axis=-axis;end
    angle=atan2d(axis(2),axis(1));scatter=sqrt(min(e));
end
function r=rot(a),r=[cos(a) -sin(a);sin(a) cos(a)];end
