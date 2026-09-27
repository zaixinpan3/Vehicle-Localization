function validateGeometryCandidate(prefix)
% validateGeometryCandidate: Independent score and raw-feature parity checks.
    if nargin<1,prefix='';end
    root=setupVehicleLocalization();addpath(root);folder=fileparts(mfilename('fullpath'));
    out=fullfile(root,'output','pole_geometry_20260927');model=loadPillarPoleModel(fullfile(out,[prefix 'candidate_model.json']));
    X=readmatrix(fullfile(out,[prefix 'replay_inputs.csv']));expected=readmatrix(fullfile(out,[prefix 'replay_expected.csv']));
    actual=zeros(size(expected));
    for first=1:500:size(X,1)
        use=first:min(first+499,size(X,1));actual(use)=scorePillarPoleModel(X(use,:),model);
    end
    assert(max(abs(actual-expected))<1e-12,'Exported model changed predictions.');
    for j=[1 size(X,1)],assert(abs(scorePillarPoleModel(X(j,:),model)-expected(j))<1e-12,'Singleton shape mismatch.');end
    assert(isempty(scorePillarPoleModel(zeros(0,size(X,2)),model)));
    predictions=readtable(fullfile(folder,[prefix 'model_predictions.csv']),'TextType','string');checks={};
    for dataset=["Mississippi","Downtown"]
        cfg=geometryCandidateConfig(dataset,prefix);cfg.voxel.useNativeKernels=true;cfg.groundSegmentation.useNativeKernels=true;cfg.offGroundFeatures.useNativeKernels=true;
        if dataset=="Mississippi",file='MissisipiPointClouds.mat';label='mississippi';frames=[44 225 285 495 720 745 776 781 850 970 1100 1170];
        else,file='downTownPointClouds.mat';label='downtown';frames=[26 86 118 165 251 282 328 356 361 411 481 536];end
        source=matfile(fullfile(root,'data','raw',file));cached=readtable(fullfile(root,'output','pole_geometry_20260927',[label '_features.csv']),'TextType','string');
        if contains(prefix,'moments')
            moment=readtable(fullfile(root,'output','pole_geometry_20260927',[label '_moments.csv']),'TextType','string');
            fields=setdiff(moment.Properties.VariableNames,{'dataset','frame','pillar','hypothesis'},'stable');cached=[cached moment(:,fields)]; %#ok<AGROW>
        end
        for frameId=frames
            frame=source.pointClouds(1,frameId);g=pillarizePointCloud(frame,cfg.voxel);ground=segmentGround(g,cfg.groundSegmentation);
            keep=~ismember(g.pointIndices,ground);p=struct('points',g.points(keep,:),'pointPillarLinIdx',g.pointPillarLinIdx(keep), ...
                'pillarGeometry',g.pillarGeometry,'pointAttributes',struct('intensity',g.pointAttributes.intensity(keep)));
            cloud=coarseSemanticProbabilityCloudConfig(cfg.voxel);cloud.semanticNames="pole";
            old=cfg.offGroundFeatures;old.pole.distributionValidation.enabled=false;off=analyzeStructuralPillars(p,old,cloud);m=off.columnMaps;
            mask=~(isfinite(p.pointAttributes.intensity) & p.pointAttributes.intensity>cfg.offGroundFeatures.trafficSignIntensityThreshold);
            [e,d]=classifyPillarPoleSupport(p.points,p.pointPillarLinIdx,g.pillarGeometry,m.poleValidationProposals, ...
                cfg.offGroundFeatures.pole.validation,mask,m.pointScore,m.lineScore,cfg.offGroundFeatures.pole.distributionValidation);
            rows=cached(cached.frame==frameId,:);assert(isequal(rows.pillar,d.owners)&&isequal(rows.hypothesis,d.hypothesisIds));
            values=rows{:,model.featureNames};values(~isfinite(values))=0;measured=d.features;measured(~isfinite(measured))=0;
            err=max(abs(values-measured)./max(1,abs(values)),[],'all');assert(err<1e-10,'Raw feature capture changed.');
            wanted=predictions.pillar(predictions.dataset==dataset & predictions.frame==frameId & predictions.score>=model.decisionThreshold);
            assert(isequal(sort(e.pillarIndices(e.found)),sort(wanted)),'Raw selection changed.');
            result=perceiveFrame(frame,cfg);selected=double(result.candidates.pillarIndices{result.candidates.semanticNames=="pole"});
            assert(isequal(sort(selected),sort(wanted)),'Pipeline selection changed.');
            checks{end+1,1}=struct('dataset',dataset,'frame',frameId,'selected',numel(selected),'maximumRelativeFeatureError',err,'sameSelections',true); %#ok<AGROW>
        end
    end
    writetable(struct2table(vertcat(checks{:})),fullfile(folder,[prefix 'raw_checks.csv']));
    value=struct('rows',size(X,1),'features',size(X,2),'maximumAbsoluteProbabilityError',max(abs(actual-expected)), ...
        'sameSelections',isequal(actual>=model.decisionThreshold,expected>=model.decisionThreshold),'rawFrames',numel(checks));
    f=fopen(fullfile(folder,[prefix 'inference_checks.json']),'w');fprintf(f,'%s\n',jsonencode(value,PrettyPrint=true));fclose(f);disp(value);
end
