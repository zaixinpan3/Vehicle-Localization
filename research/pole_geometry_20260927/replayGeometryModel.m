function replayGeometryModel(prefix)
% replayGeometryModel: Raw full-pipeline replay with frozen exact fine points.
    if nargin<1,prefix='';end
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    addpath(fullfile(root,'research','pillar_fine_alignment_20260926'));
    predictions=readtable(fullfile(folder,[prefix 'model_predictions.csv']),'TextType','string');
    for dataset=["Mississippi","Downtown"]
        label=lower(char(dataset));file='MissisipiPointClouds.mat';if dataset=="Downtown",file='downTownPointClouds.mat';end
        s=load(fullfile(root,'output','pole_precision_20260927',[label '.mat']),'frames','references');
        source=matfile(fullfile(root,'data','raw',file));cfg=geometryCandidateConfig(dataset,prefix);
        baseline=cfg;baseline.offGroundFeatures.pole.distributionValidation.enabled=false;
        rows=cell(numel(s.frames),1);before=rows;records=rows;lastBlock=-1;started=tic;
        for k=1:numel(s.frames)
            blockId=floor((s.frames(k)-1)/50);
            if blockId~=lastBlock
                first=blockId*50+1;last=min(first+49,max(s.frames));block=source.pointClouds(1,first:last);lastBlock=blockId;
            end
            frame=block(s.frames(k)-first+1);timer=tic;old=perceiveFrame(frame,baseline);oldMs=1000*toc(timer);
            timer=tic;p=perceiveFrame(frame,cfg);ms=1000*toc(timer);
            selected=double(p.candidates.pillarIndices{p.candidates.semanticNames=="pole"});
            oldIds=double(old.candidates.pillarIndices{old.candidates.semanticNames=="pole"});
            expected=predictions.pillar(predictions.dataset==dataset & predictions.frame==s.frames(k) & ...
                predictions.score>=cfg.offGroundFeatures.pole.distributionValidation.minimumScore);
            assert(isequal(sort(selected(:)),sort(expected(:))), ...
                'Raw selection differs at %s frame %d: extra %s, missing %s.',dataset,s.frames(k), ...
                mat2str(setdiff(selected,expected).'),mat2str(setdiff(expected,selected).'));
            assert(isequaln(p.sourceSummary,old.sourceSummary),'Ground/filter membership changed.');
            verifyNonPole(p.probabilityCloud.components,old.probabilityCloud.components);
            ref=s.references{k};m=measureFinePoleAlignment(frame,ref.pointIndices,selected,p.candidates.geometry);
            m.frame=s.frames(k);m.milliseconds=ms;m.nonPoleComponentsUnchanged=true;rows{k}=m;
            m=measureFinePoleAlignment(frame,ref.pointIndices,oldIds,p.candidates.geometry);
            m.frame=s.frames(k);m.milliseconds=oldMs;before{k}=m;
            records{k}=struct('frame',s.frames(k),'poleCells',selected,'baselineCells',oldIds);
            if mod(k,50)==0||k==numel(s.frames)
                T=struct2table(vertcat(rows{1:k}));B=struct2table(vertcat(before{1:k}));
                writetable(T,fullfile(folder,[prefix label '_replay.csv']));writetable(B,fullfile(folder,[label '_baseline_replay.csv']));
                fprintf('%s %d/%d %.1fs: FP %.4f, coverage %.4f\n',dataset,k,numel(s.frames),toc(started), ...
                    sum(T.extraPillarCount)/sum(T.candidatePillarCount),sum(T.coveredFinePointCount)/sum(T.finePointCountInRoi));
            end
        end
        save(fullfile(root,'output','pole_geometry_20260927',[prefix label '_replay.mat']),'records','cfg','baseline','T','B');
    end
end

function verifyNonPole(current,baseline)
    a=current.semanticName~="pole";b=baseline.semanticName~="pole";
    for name=fieldnames(current).'
        key=name{1};if any(strcmp(key,{'mixtureWeight','numComponents'})),continue;end
        x=current.(key);y=baseline.(key);
        if any(strcmp(key,{'covarianceXYZ','covariance','invCovariance'})),same=isequaln(x(:,:,a),y(:,:,b));
        else,same=isequaln(x(a,:),y(b,:));end
        assert(same,'Non-pole component changed: %s',key);
    end
end
