function screenDirection()
% screenDirection Screen fixed geometry controls without production mutation.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));addpath('output/line_direction_matching_20260928/prototype');
    s=load('output/frame601_matching_diagnosis_20260928/diagnostic.mat');rows=cell(0,8);
    for radius=[3 4 6]
        for sigma=[1 2 3 5 10]
            cfg=s.cfg.registration;cfg.lineDirection=struct('radius',radius,'minimumComponents',3,'minimumAnisotropy',9,'minimumSpan',2.4,'standardDeviation',deg2rad(sigma));
            r=experimentalDirectionRegistration(s.fixed,s.source,s.seed,cfg);e=r.poseXYTheta-s.ref;
            rows(end+1,:)={radius,sigma,r.accepted,r.reason,norm(e(1:2)),rad2deg(atan2(sin(e(3)),cos(e(3)))),r.similarity,r.observableRank}; %#ok<AGROW>
        end
    end
    results=cell2table(rows,VariableNames={'radius','sigmaDeg','accepted','reason','errorM','yawErrorDeg','similarity','rank'});
    writetable(results,fullfile(dest,'screen.csv'));disp(results);
end
