function validateRecovery()
% validateRecovery Check numerical inference, unaffected classes and timing.
    root=setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    baseline=perceptionConfig('Mississippi');
    baseline.offGroundFeatures.pole.distributionValidation=rmfield(baseline.offGroundFeatures.pole.distributionValidation,'splitShaftRecovery');
    candidate=perceptionConfig('Mississippi');
    frames=[100 300 500 685 700 820 807 827 855 856 857 858 859 900 1100];rows=cell(0,5);
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
    summary=struct('nonPoleFrames',numel(frames),'sourceMembershipUnchanged',true);
    fid=fopen(fullfile(dest,'paired_checks.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);disp(summary);
end
