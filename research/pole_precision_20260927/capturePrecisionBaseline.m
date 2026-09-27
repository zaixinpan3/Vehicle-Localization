function T=capturePrecisionBaseline()
% capturePrecisionBaseline: Current production profile on the expanded replay.
    root=setupVehicleLocalization();folder=fileparts(mfilename('fullpath'));
    addpath(fullfile(root,'research','pillar_fine_alignment_20260926'));
    out=fullfile(root,'output','pole_precision_20260927');
    s=load(fullfile(out,'downtown.mat'),'references','frames');cfg=perceptionConfig('Downtown');
    source=matfile(fullfile(root,'data','raw','downTownPointClouds.mat'));rows=cell(numel(s.frames),1);
    for k=1:numel(s.frames)
        frame=source.pointClouds(1,s.frames(k));p=perceiveFrame(frame,cfg);
        ids=double(p.candidates.pillarIndices{p.candidates.semanticNames=="pole"});
        m=measureFinePoleAlignment(frame,s.references{k}.pointIndices,ids,p.candidates.geometry);
        m.frame=s.frames(k);rows{k}=m;
    end
    T=struct2table(vertcat(rows{:}));writetable(T,fullfile(folder,'downtown_production_baseline.csv'));
    fprintf('Downtown production %d frames: %d / %d extras; %d / %d points\n',height(T), ...
        sum(T.extraPillarCount),sum(T.candidatePillarCount),sum(T.coveredFinePointCount),sum(T.finePointCountInRoi));
end
