function summary = validateMncavParameterSet()
% validateMncavParameterSet Validate the sourced model and sensor priors.
% Replay results are compared with the previous performance-evaluation
% traces. No reference data are used to choose or refit a parameter.
    root = setupVehicleLocalization();
    destination = fileparts(mfilename('fullpath'));
    output = fullfile(root,'output','mncav_parameters_20260925');
    cfg = lateralObserverConfig("mncav"); p = mncavVehicleConfig();
    sensor = mncavSensorConfig();
    saved = load(fullfile(root,'tests','reference','mncavLateralObserverDesign.mat'),'design');
    assert(isequal(saved.design.model.vehicle,p.vehicle));
    assert(abs(p.vehicle.lf+p.vehicle.lr-p.stock.wheelbaseM)<1e-12);
    assert(abs(p.vehicle.yawInertia-p.vehicle.mass* ...
        (p.stock.lengthM^2+p.stock.widthWithoutMirrorsM^2)/12)<1e-9);
    assert(sensor.referenceGnssIns.recordedImuTypeCode==41);
    assert(sensor.observerImu.nativeRateHz==50 && ~sensor.observerImu.reportedCovarianceKnown);
    assert(cfg.simulation.lateralAccelerationNoiseStd== ...
        sensor.lateralSimulation.lateralAccelerationNoiseStdMps2);
    assert(cfg.simulation.yawRateNoiseStd==sensor.lateralSimulation.yawRateNoiseStdRadps);
    previousRng = rng; cleanup = onCleanup(@()rng(previousRng));
    rows = {};
    for factor = [1 2 5]
        for seed = 2026:2045
            c = cfg; c.simulation.randomSeed = seed;
            c.simulation.lateralAccelerationNoiseStd = factor*cfg.simulation.lateralAccelerationNoiseStd;
            c.simulation.yawRateNoiseStd = factor*cfg.simulation.yawRateNoiseStd;
            result = simulateLateralObserverScenario(saved.design,c);
            target = result.truth.lateralVelocity-c.outputPoint.forwardOffsetM*result.truth.yawRate;
            error = result.estimate.lateralVelocity-target;
            post = result.truth.time>=6;
            rows(end+1,:) = {factor,seed,rms(error(post)),max(abs(error(post))), ...
                all(isfinite(result.estimate.state),'all')}; %#ok<AGROW>
        end
    end
    synthetic = cell2table(rows,VariableNames= ...
        {'noiseStdFactor','seed','rmsePost6Mps','maximumPost6Mps','finite'});
    previous = load(fullfile(root,'output','lateral_performance_20260925','experiment.mat'), ...
        'recordedTraces');
    recordedRows = {}; replays = cell(size(previous.recordedTraces));
    for k = 1:numel(previous.recordedTraces)
        trace = previous.recordedTraces{k};
        e = runLateralVelocityObserver(trace.high,saved.design,cfg);
        delta = max(abs(e.lateralVelocity-trace.estimate.lateralVelocity));
        assert(delta==0,'The metadata integration unexpectedly changed recorded replay.');
        error = e.lateralVelocity-trace.reference(:,2);
        recordedRows(end+1,:) = {trace.drive,numel(error),rms(error),mean(error),delta}; %#ok<AGROW>
        replays{k} = e;
    end
    recorded = cell2table(recordedRows,VariableNames= ...
        {'drive','samples','rmseMps','biasMps','maxDifferenceFromPriorReplayMps'});
    results = runtests({'tests/mncavVehicleConfigTest.m','tests/lateralObserverTest.m', ...
        'tests/wheelLongitudinalSpeedTest.m','tests/wheelMotionInputTest.m'});
    tests = table(results);
    writetable(tests(:,1:5),fullfile(destination,'tests.csv'));
    assert(all([results.Passed]),'All selected tests, including synthesis, must execute and pass.');
    writetable(synthetic,fullfile(destination,'synthetic_validation.csv'));
    writetable(recorded,fullfile(destination,'recorded_validation.csv'));
    summary = struct('matlab',version,'testCount',numel(results), ...
        'syntheticRuns',height(synthetic),'recorded',recorded, ...
        'dynamicsUnchanged',true,'gainResynthesisRequired',false);
    save(fullfile(output,'validation.mat'),'summary','synthetic','recorded','replays','cfg');
    disp(recorded); disp(summary);
end
