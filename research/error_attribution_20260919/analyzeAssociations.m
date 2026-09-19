function analyzeAssociations()
% analyzeAssociations Freeze pair identities and whitening at known reference.
% An offline oracle distinguishes association changes from residual-model bias.
% This is intentionally not a replacement solver or an online confidence gate.
    setupVehicleLocalization();
    s=load('output/error_attribution_20260919/diagnostics.mat');
    cfg=distributionRegistrationConfig(); rows=cell(0,7); exported=cell(0,1);
    for k=1:numel(s.snapshots)
        a=s.snapshots{k}; if a.mode~="reference", continue; end
        c=a.source.components; f=a.map.components; p=a.result.correspondences;
        pose=a.reference; rot=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];
        rotated=pagemtimes(pagemtimes(rot,c.covariance),rot.');
        target=f.mean(p.target,:)-pose(1:2); source=c.mean(p.source,:);
        whitening=zeros(2,2,height(p));
        for j=1:height(p)
            covariance=rotated(:,:,p.source(j))+f.covariance(:,:,p.target(j))+cfg.geometric.noiseStandardDeviation^2*eye(2);
            if any(p.semanticName(j)==["curb","facade"])
                [v,d]=eig(f.covariance(:,:,p.target(j)),'vector');[~,ix]=min(d);normal=v(:,ix);
                whitening(1,:,j)=normal.'/sqrt(normal.'*covariance*normal);
            else
                whitening(:,:,j)=chol(covariance,'lower')\eye(2);
            end
        end
        for width=[3 2]
            objective=@(q) frozenScore(q,source,target,whitening,pose(3),p.weight,cfg);
            [q,score,flag]=fminsearch(objective,zeros(width,1),optimset('Display','off','MaxIter',3000,'TolX',1e-8,'TolFun',1e-10));
            yaw=0; if width==3,yaw=rad2deg(q(3)/cfg.yawLeverArm);end
            rows(end+1,:)={a.frame,width,norm(q(1:2)),yaw,objective(zeros(width,1)),score,flag}; %#ok<AGROW>
        end
        if ismember(a.frame,[92 820 830])
            b=a; b.map.components.mean=(f.mean-pose(1:2))*rot;
            b.map.components.covariance=pagemtimes(pagemtimes(rot.',f.covariance),rot);
            keep=vecnorm(b.map.components.mean,2,2)<40;
            entries=struct('frame',a.frame,'sourceMean',c.mean,'sourceName',c.semanticName, ...
                'targetMean',b.map.components.mean(keep,:),'targetName',f.semanticName(keep), ...
                'targetCovariance',b.map.components.covariance(:,:,keep), ...
                'pairedSource',c.mean(p.source,:),'pairedTarget',b.map.components.mean(p.target,:), ...
                'pairClass',p.semanticName,'pose',pose);
            exported{end+1}=entries; %#ok<AGROW>
        end
    end
    results=cell2table(rows,VariableNames={'frame','freeDimensions','positionErrorM','yawErrorDeg','initialScore','finalScore','exitflag'});
    writetable(results,'output/error_attribution_20260919/frozen_pairs.csv');disp(results);
    fid=fopen('output/error_attribution_20260919/association_geometry.json','w');cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'%s',jsonencode(exported));
end

function score=frozenScore(q,source,target,whitening,yaw,weights,cfg)
    if numel(q)==3, yaw=yaw+q(3)/cfg.yawLeverArm; end
    rot=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
    delta=source*rot.'+q(1:2).'-target;
    residual=pagemtimes(whitening,reshape(delta.',2,1,[]));
    distance=reshape(sum(residual.^2,1),[],1); scale=cfg.geometric.robustStandardizedDistance^2;
    score=sum(weights.*scale.*log1p(distance/scale));
end
