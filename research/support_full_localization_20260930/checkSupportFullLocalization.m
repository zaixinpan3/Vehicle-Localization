function checkSupportFullLocalization()
% checkSupportFullLocalization Check replay equations and causal input use.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    folder='output/support_full_localization_20260930';
    b=load(fullfile(folder,'observer','experiment.mat'));r=b.runs{1}.estimate;
    assertMncavReplayCurrent(b.report.metadata.inputMetadata.parameters);
    assert(isequaln(b.lateralDesign.cfg.vehicle,lateralObserverConfig().vehicle));
    repeat=runFullLocalizationObserver(b.data,b.lateralDesign,b.cfg,LateralInputs=b.lateral);
    repeatDifference=max(abs(repeat.z-r.z),[],'all');assert(repeatDifference==0);
    % The synchronous algorithm makes one implicit frame update, so the old
    % continuous integrator setting must not affect this execution branch.
    cfg=b.cfg;cfg.maximumIntegrationStep=cfg.maximumIntegrationStep/2;
    refined=runFullLocalizationObserver(b.data,b.lateralDesign,cfg,LateralInputs=b.lateral);
    stepDifference=max(abs(refined.z-r.z),[],'all');assert(stepDifference==0);
    dt=diff(r.time);p=r.position;
    residual=max(abs(diff(p)-dt.*(r.velocity(2:end,:)+r.diagnostics.positionCorrection(2:end,1:2)+ ...
        r.diagnostics.positionCorrection(2:end,3:4))),[],'all');
    assert(residual<1e-7);
    data=b.data;data.gnss.position(data.gnss.time>60,:)=100;
    data.lidar.pose(data.lidar.time>60 & data.lidar.valid,:)=100;
    mutated=runFullLocalizationObserver(data,b.lateralDesign,b.cfg,LateralInputs=b.lateral);
    prefixDifference=max(abs(mutated.z(r.time<=60,:)-r.z(r.time<=60,:)),[],'all');assert(prefixDifference==0);
    assert(r.diagnostics.virtualPoseUpdates==0 && r.diagnostics.stateResets==0 && r.diagnostics.integrationSubsteps==0);
    assert(~r.diagnostics.referenceUsed);
    assert(all(cellfun(@(x)isequal(x.estimate.z(1,:),r.z(1,:)),b.runs)));
    for k=1:numel(b.runs),assert(all(isfinite(b.runs{k}.estimate.z),'all'));end
    tests=readtable(fullfile(dest,'tests.csv'),TextType='string');
    assert(~any(tests.incomplete));
    failed=tests.test(logical(tests.failed));
    assert(isequal(failed,"lateralObserverTest/currentDesignSatisfiesTheOriginalCertificate"), ...
        'Reassess changed regression outcomes; do not silently accept another failure.');
    % Preserve the existing test failure rather than changing unrelated work.
    % Inspect the actual run's original certificate independently; its tiny
    % recovered gain can exceed an almost-zero optimization bound numerically.
    design=b.lateralDesign;worst=-Inf;worstNoise=-Inf;
    for point=design.grid.points.'
        [L,P]=scheduleLateralObserverGain(design,point.speed);
        Pdot=point.alphaRate(1)*design.lyapunovBasis(:,:,1)+point.alphaRate(2)*design.lyapunovBasis(:,:,2)+point.alphaRate(3)*design.lyapunovBasis(:,:,3);
        A=point.A-L*point.C;
        Psi=P*A+A.'*P+Pdot+2*design.decayRate*P+design.tau*(P*P)+design.lipschitzConstant^2/design.tau*eye(2);
        noise=[Psi,-P*L;-(P*L).',-design.issGain^2*eye(2)];
        worst=max(worst,max(eig((Psi+Psi.')/2)));worstNoise=max(worstNoise,max(eig((noise+noise.')/2)));
    end
    assert(worst<0 && worstNoise<=1e-9);
    gainDifference=design.maxVertexGainNorm-design.gainBound*(1+1e-6);
    assert(gainDifference>0 && gainDifference<1e-8);
    a=load(fullfile(folder,'matching','report.mat'),'report');
    assert(height(a.report.calls)==1170 && a.report.metadata.perceptionRerun && ~a.report.metadata.finePerceptionUsed);
    assert(a.report.metadata.registrationMethod=="supportD2D");
    audit=jsondecode(fileread(fullfile(folder,'call_path_audit.json')));
    assert(audit.coarseEntryExecuted && audit.fineRefinementCalls==0);
    validation=struct('tests',height(tests),'testSuites',11,'allTestsPassed',all(tests.passed), ...
        'passedTests',nnz(tests.passed),'failedTests',failed,'runtimeChecksPassed',true, ...
        'lateralOriginalCertificateMaximumEigenvalue',worst,'lateralNoiseCertificateMaximumEigenvalue',worstNoise, ...
        'lateralRecoveredGainNorm',design.maxVertexGainNorm,'lateralReportedGainBound',design.gainBound, ...
        'lateralGainBoundExcess',gainDifference,'lateralGainBoundCheckResolved',false, ...
        'repeatMaximumStateDifference',repeatDifference,'integrationSettingStateDifference',stepDifference, ...
        'implicitPositionEquationMaximumResidualM',residual,'futureMutationPrefixDifference',prefixDifference, ...
        'allScenariosFinite',true,'initialStateCommon',true,'rawFrames',1170,'observerFrames',numel(r.time), ...
        'coarseCallPathAudited',true,'fineRefinementCalls',0,'vehicleParametersCurrent',true, ...
        'initializationReferenceBased',true,'runtimeReferenceUsed',r.diagnostics.referenceUsed, ...
        'zeroProcessingDelay',true,'offlineSynchronization',true,'sampledSystemCertified',false);
    fid=fopen(fullfile(dest,'validation.json'),'w');fprintf(fid,'%s\n',jsonencode(validation,PrettyPrint=true));fclose(fid);
    disp(validation);
end
