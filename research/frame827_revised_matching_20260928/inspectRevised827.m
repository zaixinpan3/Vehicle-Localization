function inspectRevised827()
% inspectRevised827 Export actual canonical associations and replay checks.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    a=load('output/frame827_revised_matching_20260928/results.mat');
    s=load('output/line_direction_matching_20260928/sources.mat','fixed');
    [fixed,ids]=selectLocalProbabilityCloud(s.fixed,a.predicted,a.reg.localMapRadius);
    r=a.results{3};mapRadius=0;sourceRadius=0;
    if r.pyramid.coarseRetained,mapRadius=a.reg.pyramid.mapMergeRadius;sourceRadius=a.reg.pyramid.sourceMergeRadius;end
    [fixed,groups]=canonicalizeSemanticCloud(fixed,mapRadius,a.reg.pyramid.pointClasses);
    [source,sourceGroups]=canonicalizeSemanticCloud(a.source,sourceRadius,a.reg.pyramid.pointClasses);
    mdl=prepareSemanticRegistrationGeometry(fixed,source,a.predicted,a.reg);pose=r.poseXYTheta;
    sys=mdl.linearize([pose(1:2)-a.predicted(1:2) pose(3)],[1;1;1/a.reg.yawLeverArm]);
    pairs=struct2table(sys.pairs);pairs.sourceBody=source.components.mean(pairs.source,:);
    R=[cos(a.ref(3)) -sin(a.ref(3));sin(a.ref(3)) cos(a.ref(3))];
    pairs.targetReferenceBody=(fixed.components.mean(pairs.target,:)-a.ref(1:2))*R;
    pairs.referenceCenterDistance=vecnorm(pairs.sourceBody-pairs.targetReferenceBody,2,2);
    pairs.mapMembers=cellfun(@(x)strjoin(string(ids(x)),";"),groups(pairs.target));
    pairs.sourceMembers=cellfun(@(x)strjoin(string(x),";"),sourceGroups(pairs.source));
    writetable(pairs,fullfile(dest,'solution_pairs.csv'));
    e=pose-a.ref;d=a.predicted-a.ref;
    checks=checkcode(fullfile(dest,'evaluateRevised827.m'),'-config=factory');
    checks2=checkcode(fullfile(dest,'inspectRevised827.m'),'-config=factory');
    summary=struct('frame',827,'rawFramesReplayed',height(a.replay),'positionErrorM',norm(e(1:2)), ...
        'bodyErrorM',e(1:2)*R,'predictionErrorM',norm(d(1:2)), ...
        'predictionYawErrorDeg',rad2deg(atan2(sin(d(3)),cos(d(3)))), ...
        'matchingMs',a.replay.matchingMs(end),'matchedPairs',height(pairs), ...
        'codeAnalyzerFindings',numel(checks)+numel(checks2),'productionChanged',false);
    assert(height(a.replay)==827&&all(isfinite(a.replay.errorM)));
    fid=fopen(fullfile(dest,'summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summary,PrettyPrint=true));fclose(fid);
    disp(summary);disp(r.classDiagnostics);disp(pairs(pairs.semanticName=="pole",:));
end
