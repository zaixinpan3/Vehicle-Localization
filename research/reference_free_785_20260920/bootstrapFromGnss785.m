function bootstrap=bootstrapFromGnss785(map,cloud,gnss,cfg,registration)
% bootstrapFromGnss785 Search heading using one GNSS packet and local features.
% No recorded reference position or attitude is an argument. Search the full
% heading circle at fixed ten-degree spacing, selecting accepted similarity.
    assert(gnss.valid(1),'A valid first GNSS packet is required.');
    headings=deg2rad((-180:10:170).');n=numel(headings);rows=cell(n,10);runs=cell(n,1);
    for k=1:n
        yaw=headings(k);
        position=correctGnssOutputPoint(gnss.position(1,:),gnss.information(:,:,1),yaw,cfg.gnss.outputPoint);
        seed=[position,yaw];r=registerSemanticProbabilityCloud(map,cloud,seed,registration);runs{k}=r;
        rows(k,:)={k,yaw,seed(1),seed(2),r.accepted,string(r.reason),r.similarity, ...
            r.poseXYTheta(1),r.poseXYTheta(2),r.poseXYTheta(3)};
    end
    candidates=cell2table(rows,VariableNames={'candidate','initialPsi','initialX','initialY', ...
        'accepted','reason','similarity','x','y','psi'});
    eligible=find(candidates.accepted);assert(~isempty(eligible),'No accepted GNSS/map initialization.');
    [~,best]=max(candidates.similarity(eligible));selected=eligible(best);
    bootstrap=struct('result',runs{selected},'initialPose', ...
        candidates{selected,{'initialX','initialY','initialPsi'}}, ...
        'selected',selected,'candidates',candidates,'candidateResults',{runs});
end
