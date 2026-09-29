function validateSpecialist()
% validateSpecialist Check numerical inference, unaffected classes and timing.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));out='output/pole_selective_recovery_20260928';
    model=loadPillarPoleModel(fullfile(root,out,'model.json'));
    x=readmatrix(fullfile(out,'inference_inputs.csv'));expected=readmatrix(fullfile(out,'inference_expected.csv'));
    actual=scorePillarPoleModel(x,model);assert(max(abs(actual-expected))<1e-12);
    baseline=perceptionConfig('Mississippi');
    baseline.offGroundFeatures.pole.distributionValidation.modelFile=fullfile(root,'config/polePillarDistributionModel.json');
    baseline.offGroundFeatures.pole.distributionValidation.minimumScore=.8701683227450074;
    baseline.offGroundFeatures.pole.distributionValidation=rmfield(baseline.offGroundFeatures.pole.distributionValidation,'minorityShaftProtection');
    candidate=perceptionConfig('Mississippi');
    candidate.offGroundFeatures.pole.distributionValidation.modelFile=fullfile(root,out,'model.json');
    candidate.offGroundFeatures.pole.distributionValidation.minimumScore=model.decisionThreshold;
    frames=[100 300 500 601 700 807 827 855 856 857 858 859 900 1100];rows=cell(0,5);
    for frame=frames
        raw=loadPointCloudFrame(fullfile(root,'data/raw/MissisipiPointClouds.mat'),frame);
        for repeat=1:3
            configs={baseline,candidate};order=1:2;if mod(repeat,2)==0,order=2:-1:1;end
            values=cell(1,2);times=zeros(1,2);
            for v=order,t=tic;values{v}=perceiveFrame(raw,configs{v});times(v)=1000*toc(t);end
            for name=["curb","trafficSign"]
                a=values{1}.candidates;b=values{2}.candidates;
                assert(isequal(a.pillarIndices{a.semanticNames==name},b.pillarIndices{b.semanticNames==name}));
            end
            assert(isequaln(values{1}.sourceSummary,values{2}.sourceSummary));
            for v=1:2,rows(end+1,:)={frame,repeat,v,times(v),numel(values{v}.candidates.pillarIndices{values{v}.candidates.semanticNames=="pole"})};end %#ok<AGROW>
        end
    end
    writetable(cell2table(rows,VariableNames={'frame','repeat','variant','milliseconds','polePillars'}),fullfile(dest,'paired_runtime.csv'));
    summary=struct('inferenceRows',size(x,1),'maxInferenceDifference',max(abs(actual-expected)), ...
        'nonPoleFrames',numel(frames),'sourceMembershipUnchanged',true);
    fid=fopen(fullfile(dest,'inference_checks.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);disp(summary);
end
