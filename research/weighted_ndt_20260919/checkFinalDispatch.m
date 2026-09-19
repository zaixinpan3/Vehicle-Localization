function checkFinalDispatch()
setupVehicleLocalization;maxNumCompThreads(8);
s=load('output/stability_ndt_20260919/probe.mat');
c=readtable('output/stability_ndt_20260919/overlap_full/recursive/calls.csv','TextType','string');
rows=cell(numel(s.frames),5);
for n=1:numel(s.frames)
    k=s.frames(n);initial=[c.predictedX(k),c.predictedY(k),c.predictedPsi(k)];
    r=registerSemanticProbabilityCloud(s.map,s.clouds{n},initial,weightedNdtRegistrationConfig());
    information=r.information;expected=[c.informationXX(k) c.informationXY(k) c.informationXPsi(k);c.informationXY(k) c.informationYY(k) c.informationYPsi(k);c.informationXPsi(k) c.informationYPsi(k) c.informationPsiPsi(k)];
    actual=initial;
    if r.accepted,actual=r.poseXYTheta;end
    reference=[c.x(k),c.y(k),c.psi(k)];
    rows(n,:)={k,r.reason,r.reason==c.reason(k),norm(actual-reference),norm(information-expected,'fro')};
end
checks=cell2table(rows,VariableNames={'frame','reason','sameDecision','poseDifference','informationDifference'});
disp(checks);writetable(checks,'research/weighted_ndt_20260919/dispatch_checks.csv');
assert(all(checks.sameDecision));assert(max(checks.poseDifference)<1e-6);assert(max(checks.informationDifference)<1e-5);
end
