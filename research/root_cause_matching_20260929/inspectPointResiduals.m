function inspectPointResiduals()
% inspectPointResiduals Export actual feature residuals in the reference frame.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    b=load('output/pole_boundary_recovery_20260929/replay.mat','sources');
    cfg=distributionRegistrationConfig();mc=featureMapBuildConfig();a=load(mc.probabilityCloudPath,'cloud');
    for k=[178 806 854 855 893]
        calls=load('output/line_direction_matching_20260928/production/report.mat','report');
        ref=calls.report.calls{k,{'referenceX','referenceY','referencePsi'}};
        fixed=selectLocalProbabilityCloud(a.cloud,ref,cfg.localMapRadius);r=registerSemanticProbabilityCloud(fixed,b.sources{k},ref,cfg);
        t=r.correspondences;R=[cos(ref(3)) -sin(ref(3));sin(ref(3)) cos(ref(3))];
        t.mapBody=(t.targetMeanXY-ref(1:2))*R;t.trueResidualXY=t.sourceMeanXY-t.mapBody;
        writetable(t,fullfile(dest,"point_residuals_"+k+".csv"));
        disp(k);disp(r.pyramid);disp(t(t.semanticName~="curb",:));
    end
end
