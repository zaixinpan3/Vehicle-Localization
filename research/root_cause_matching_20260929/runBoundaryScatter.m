function runBoundaryScatter()
% runBoundaryScatter Propagate boundary scatter instead of road/shoulder mixing.
% The fitted boundary centers determine a local normal spread. Tangential and
% height scatter are retained through a congruence transform, preserving PSD.
    setupVehicleLocalization();out='output/root_cause_matching_20260929';
    files=['output/pole_boundary_recovery_20260929/replay.mat',"output/root_cause_matching_20260929/curbSurface_sources.mat"];
    old=load('output/line_direction_matching_20260928/production/report.mat','report');o=load('output/line_direction_matching_20260928/sources.mat','motion');cfg=distributionRegistrationConfig();wc=localizationSourceWindowConfig();
    for variant=1:2
        b=load(files(variant),'currentClouds');currentClouds=b.currentClouds;sources=cell(1170,1);history=[];
        for k=1:1170
            c=currentClouds{k}.components;[tangent,valid]=sourceLineDirections(c,cfg.lineDirection);
            for id=find(valid).'
                t=tangent(id,:).';n=[-t(2);t(1)];near=c.semanticName==c.semanticName(id) & sum((c.mean-c.mean(id,:)).^2,2)<=cfg.lineDirection.radius^2;
                delta=c.mean(near,:)-mean(c.mean(near,:),1);normalVariance=mean((delta*n).^2);
                original=n.'*c.covariance(:,:,id)*n;desired=max(.01,min(original,normalVariance));L=t*t.'+sqrt(desired/original)*(n*n.');
                S=L*c.covariance(:,:,id)*L.';c.covariance(:,:,id)=(S+S.')/2;c.invCovariance(:,:,id)=S\eye(2);c.determinant(id)=det(S);c.logNormalizationConstant(id)=-log(2*pi)-.5*log(det(S));
                L3=blkdiag(L,1);c.covarianceXYZ(:,:,id)=L3*c.covarianceXYZ(:,:,id)*L3.';
            end
            currentClouds{k}.components=c;[sources{k},history]=updateLocalizationSourceWindow(currentClouds{k},old.report.calls.timeSeconds(k),o.motion(k,:),history,wc);
        end
        label="boundaryScatter"+variant;file=fullfile(out,label+"_sources.mat");save(file,'sources','currentClouds','-v7.3');rootCauseReplay(file,label,cfg);
        c=cfg;c.geometric.robustLoss="switchable";c.geometric.classGateNormalization="global";rootCauseReplay(file,label+"Bounded",c);
    end
end
