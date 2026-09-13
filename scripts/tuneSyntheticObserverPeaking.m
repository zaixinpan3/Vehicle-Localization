function report = tuneSyntheticObserverPeaking(outputFolder)
% tuneSyntheticObserverPeaking Compare constant certified GNSS gain settings.
% Reuses the exact clean/noisy sedan input, truth and initial errors from
% demoSyntheticVehicleObserver. Does not clip states or alter the trajectory.
    arguments
        outputFolder (1,1) string = "output/observer_peaking_tuning"
    end
    setupVehicleLocalization;
    baselineFile = "output/synthetic_vehicle_observer/traces.mat";
    if ~isfile(baselineFile), demoSyntheticVehicleObserver; end
    stored = load(baselineFile,'results');
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    % name, theta, position/velocity/acceleration chain coefficients, yaw gain
    candidates = {"reference",20,[3;3;1],.5; ...
        "theta16",16,[3;3;1],.5; "theta14",14,[3;3;1],.5; ...
        "theta12",12,[3;3;1],.5; "theta10_rejected",10,[3;3;1],.5; ...
        "shaped8",8,[6;8;3],.5; "shaped8_yaw01",8,[6;8;3],.1; ...
        "shaped8_yaw02",8,[6;8;3],.2; "shaped8b",8,[8;12;4],.2};
    rows = cell(size(candidates,1)*2,1);
    runs = cell(size(rows));
    index = 0;
    for k = 1:size(candidates,1)
        for j = 1:2
            index = index+1;
            baseline = stored.results{j};
            cfg = baseline.cfg;
            cfg.observer.theta = candidates{k,2};
            cfg.observer.yawGain = candidates{k,4};
            design = candidateDesign(cfg,candidates{k,3});
            row = struct('name',candidates{k,1},'noisy',baseline.noisy, ...
                'theta',cfg.observer.theta,'yawGain',cfg.observer.yawGain, ...
                'kPosition',candidates{k,3}(1),'kVelocity',candidates{k,3}(2), ...
                'kAcceleration',candidates{k,3}(3), ...
                'certificateMargin',design.verification.uniformMargin, ...
                'certificatePassed',design.verification.certified, ...
                'peakVelocityErrorMps',NaN,'peakAccelerationErrorMps2',NaN, ...
                'peakHeadingErrorDeg',NaN,'positionRmseM',NaN,'velocityRmseMps',NaN, ...
                'accelerationRmseMps2',NaN,'headingRmseDeg',NaN, ...
                'settlingTimeSec',NaN,'passed',false);
            if design.verification.certified
                % Use identical actual upstream outputs for every candidate.
                estimate = runImprovedVehicleObserver(baseline.sensorData,struct(),design,cfg, ...
                    LateralInputs=baseline.estimate.lateral);
                e = estimate.z-baseline.truth;
                pe = vecnorm(e(:,[1,4]),2,2);ve = vecnorm(e(:,[2,5]),2,2);
                ae = vecnorm(e(:,[3,6]),2,2);he = rad2deg(abs(e(:,7)));
                settled = baseline.time>=20;
                row.peakVelocityErrorMps = max(ve);
                row.peakAccelerationErrorMps2 = max(ae);
                row.peakHeadingErrorDeg = max(he);
                row.positionRmseM = rms(pe(settled));row.velocityRmseMps = rms(ve(settled));
                row.accelerationRmseMps2 = rms(ae(settled));row.headingRmseDeg = rms(he(settled));
                inside = pe<=.15 & ve<=.15 & ae<=.15 & he<=1;
                last = find(~inside,1,'last');
                if isempty(last), row.settlingTimeSec=0;
                elseif last<numel(pe), row.settlingTimeSec=baseline.time(last+1); end
                row.passed = all(isfinite(estimate.z),'all') && row.positionRmseM<=.15 ...
                    && row.velocityRmseMps<=.15 && row.accelerationRmseMps2<=.15 ...
                    && row.headingRmseDeg<=1;
                runs{index} = struct('estimate',estimate,'cfg',cfg,'design',design);
            end
            rows{index} = row;
        end
    end
    report.metrics = struct2table([rows{:}]);
    report.scope = "Same 40 s synthetic inputs and initialization; fixed coefficients and original operating bounds.";
    report.recommendedCfg = improvedObserverConfig("gnss","lowPeaking");
    report.recommendedDesign = improvedObserverReferenceDesign(report.recommendedCfg);
    selected = find(report.metrics.name=="shaped8_yaw01" & report.metrics.noisy);
    baseline = stored.results{2};
    selectedRun = runs{selected};
    assert(norm(report.recommendedDesign.K-selectedRun.design.K,'fro')<1e-12);
    assert(norm(report.recommendedDesign.N-selectedRun.design.N,'fro')<1e-12);
    assert(norm(report.recommendedDesign.P-selectedRun.design.P,'fro')<1e-12);
    refinedCfg = selectedRun.cfg;
    refinedCfg.measurement.maximumIntegrationStep = .0025;
    refined = runImprovedVehicleObserver(baseline.sensorData,struct(),selectedRun.design,refinedCfg, ...
        LateralInputs=baseline.estimate.lateral);
    report.maximumStateRefinementDifference = max(abs(refined.z-selectedRun.estimate.z),[],1);
    report.referenceReproductionMaximumDifference = max(abs(runs{2}.estimate.z-baseline.estimate.z),[],'all');
    report.accelerationPeakReductionPercent = 100*(1- ...
        report.metrics.peakAccelerationErrorMps2(selected)/report.metrics.peakAccelerationErrorMps2(2));
    drawComparison(baseline,runs{2},selectedRun,outputFolder);
    writetable(report.metrics,fullfile(outputFolder,'metrics.csv'));
    save(fullfile(outputFolder,'traces.mat'),'runs','report','-v7.3');
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);
    cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    disp(report.metrics);
end

function drawComparison(baseline,reference,selected,outputFolder)
    t=baseline.time;
    errors={reference.estimate.z-baseline.truth,selected.estimate.z-baseline.truth};
    fig=figure('Name','GNSS startup gain tuning','Color','w','Position',[100,100,1100,700]);
    tiledlayout(fig,2,2,'TileSpacing','compact');
    names=["Position error (m)","Velocity error (m/s)", ...
        "Acceleration error (m/s^2)","Heading error (deg)"];
    indices={[1,4],[2,5],[3,6],7};
    for k=1:4
        nexttile;hold on;
        for j=1:2
            value=vecnorm(errors{j}(:,indices{k}),2,2);
            if k==4,value=rad2deg(value);end
            plot(t,value,'LineWidth',1.4);
        end
        xlim([0,5]);grid on;xlabel('Time (s)');ylabel(names(k));
        legend('Reference','Low peaking','Location','best');
    end
    sgtitle('Identical noisy inputs and initial errors; constant GNSS gains');
    exportgraphics(fig,fullfile(outputFolder,'startup_comparison.png'),'Resolution',150);
    exportgraphics(fig,fullfile(outputFolder,'startup_comparison.pdf'),'ContentType','vector');
end

function design = candidateDesign(cfg,chainGain)
    original = improvedObserverConfig("gnss");
    design = improvedObserverReferenceDesign(original);
    design.theta = cfg.observer.theta;
    design.K = [chainGain,zeros(3,1);zeros(3,1),chainGain;zeros(1,2)];
    design.N(7,4) = -cfg.observer.yawGain*design.theta^2;
    data = buildImprovedObserverCertificateData(cfg);
    F = data.A(1:6,1:6)-design.K(1:6,:)*data.Cg;
    identity = eye(6);
    operator = kron(identity,F.')+kron(F.',identity);
    design.P = reshape(operator\(-identity(:)),6,6);
    design.P = (design.P+design.P.')/2;
    design.verification = verifyImprovedObserverDesign(design,cfg);
    design.certified = design.verification.certified;
end
