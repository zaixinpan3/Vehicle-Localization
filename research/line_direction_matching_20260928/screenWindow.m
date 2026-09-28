function screenWindow()
% screenWindow Test causal support length and direction strength on cached scans.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));addpath('output/line_direction_matching_20260928/prototype');
    s=load('output/frame601_matching_diagnosis_20260928/diagnostic.mat');rows=cell(0,10);
    for horizon=[3 5 8 10 15]
        window=s.cfg.sourceWindow;window.maximumFrames=horizon;window.maximumAgeSeconds=.1*horizon-.04;
        history=[];
        for k=1:find(s.frames==601)
            f=s.frames(k);d=s.baseline.report.deadReckoning(f,:);c=s.baseline.report.calls(f,:);
            [source,history]=updateLocalizationSourceWindow(s.currents{k},c.timeSeconds,[d.x d.y d.psi],history,window);
        end
        for sigma=[.25 .5 1 2]
            cfg=s.cfg.registration;cfg.lineDirection=struct('radius',4,'minimumComponents',3,'minimumAnisotropy',9,'minimumSpan',2.4,'standardDeviation',deg2rad(sigma));
            r=experimentalDirectionRegistration(s.fixed,source,s.seed,cfg);e=r.poseXYTheta-s.ref;
            rows(end+1,:)={horizon,sigma,r.accepted,r.reason,norm(e(1:2)),rad2deg(atan2(sin(e(3)),cos(e(3)))),r.similarity,r.observableRank,nnz(source.components.semanticName=="curb"),nnz(source.components.semanticName=="pole")}; %#ok<AGROW>
        end
    end
    results=cell2table(rows,VariableNames={'horizon','sigmaDeg','accepted','reason','errorM','yawErrorDeg','similarity','rank','curb','pole'});
    writetable(results,fullfile(dest,'window_screen.csv'));disp(results);
end
