function T=captureContextAlignment(dataset,frames,label,validationCfg)
% captureContextAlignment: Score production against frozen original fine points.
% Validate every non-pole component except its globally normalized weight.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    out=fullfile(root,'output','pole_context_20260927');
    referenceFolder=fullfile(root,'output','pillar_fine_alignment_20260926');
    addpath(fullfile(root,'research','pillar_fine_alignment_20260926'));
    cfg=perceptionConfig(dataset);rows=cell(numel(frames),1);records=rows;
    if nargin>=4,cfg.offGroundFeatures.pole.validation=validationCfg;end
    if strcmpi(dataset,'Downtown')
        reference=load(fullfile(referenceFolder,'downtown_reference.mat'),'references','frames');
        baseline=load(fullfile(root,'output','pillar_shaft_20260926','downtown_pipeline.mat'),'records','frames');
        source=matfile(fullfile(root,'data','raw','downTownPointClouds.mat'));
    else
        reference=load(fullfile(referenceFolder,'reference.mat'),'references');reference.frames=1:1170;
        baseline=load(fullfile(root,'output','pillar_shaft_20260926','full_pipeline.mat'),'records');baseline.frames=1:1170;
        source=matfile(fullfile(root,'data','raw','MissisipiPointClouds.mat'));
    end
    for k=1:numel(frames)
        if mod(k-1,50)==0
            selectedFrames=frames(k:min(k+49,numel(frames)));
            block=[];
            if numel(selectedFrames)<3 || all(diff(selectedFrames)==selectedFrames(2)-selectedFrames(1))
                block=source.pointClouds(1,selectedFrames);
            end
        end
        if isempty(block),frame=source.pointClouds(1,frames(k));else,frame=block(mod(k-1,50)+1);end
        timer=tic;p=perceiveFrame(frame,cfg);milliseconds=1000*toc(timer);
        ref=reference.references{find(reference.frames==frames(k),1)};
        old=baseline.records{find(baseline.frames==frames(k),1)};
        ids=double(p.candidates.pillarIndices{p.candidates.semanticNames=="pole"});
        m=measureFinePoleAlignment(frame,ref.pointIndices,ids,p.candidates.geometry);
        m.frame=frames(k);m.milliseconds=milliseconds;
        assert(isequaln(p.sourceSummary,old.sourceSummary),'Ground/filter membership changed.');
        verifyNonPoleComponents(p.probabilityCloud.components,old.components);
        m.nonPoleComponentsUnchanged=true;rows{k}=m;
        records{k}=struct('frame',frames(k),'poleCells',ids);
        if mod(k,50)==0 || k==numel(frames)
            T=struct2table(vertcat(rows{1:k}));T=movevars(T,'frame','Before',1);
            writetable(T,fullfile(folder,[label '_frames.csv']));processed=k;
            save(fullfile(out,[label '.mat']),'cfg','frames','records','T','processed','-v7.3');
            fprintf('%s %d/%d: coverage %.4f, precision %.4f, pillar recall %.4f\n',label,k,numel(frames), ...
                sum(T.coveredFinePointCount)/sum(T.finePointCountInRoi), ...
                sum(T.coveredFinePillarCount)/sum(T.candidatePillarCount), ...
                sum(T.coveredFinePillarCount)/sum(T.finePillarCount));
        end
    end
end

function verifyNonPoleComponents(current,baseline)
    a=current.semanticName~="pole";b=baseline.semanticName~="pole";
    for name=fieldnames(current).'
        key=name{1};if any(strcmp(key,{'mixtureWeight','numComponents'})),continue;end
        x=current.(key);y=baseline.(key);
        if any(strcmp(key,{'covarianceXYZ','covariance','invCovariance'}))
            equal=isequaln(x(:,:,a),y(:,:,b));
        else
            equal=isequaln(x(a,:),y(b,:));
        end
        assert(equal,'Non-pole component changed: %s',key);
    end
end
