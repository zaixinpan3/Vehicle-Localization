function results = diagnoseMncavObserverOutage(sequenceFolder, outputFolder)
% diagnoseMncavObserverOutage Isolate measurement, gain and integration effects.
% These controlled counterfactuals are diagnostics, not certified designs.
% Production observer sources and recorded D2D poses/information are unchanged.
    arguments
        sequenceFolder (1,1) string
        outputFolder (1,1) string
    end
    if ~isfolder(outputFolder), mkdir(outputFolder); end
    designs=load(fullfile(sequenceFolder,'mncavObserverDesign.mat'));
    calls=readtable(fullfile(sequenceFolder,'recursive_8threads','calls.csv'));
    [inputs,reference]=prepareMncavObserverReplay(fullfile(sequenceFolder,'sensors'), ...
        fullfile(sequenceFolder,'vehicle_parameters.json'),calls,.15);
    keep=~(inputs.gps.timestamp>=40 & inputs.gps.timestamp<60);
    inputs.gps.timestamp=inputs.gps.timestamp(keep);
    inputs.gps.arrivalTime=inputs.gps.arrivalTime(keep);
    inputs.gps.pose=inputs.gps.pose(keep,:);
    cfg=designs.observerCfg;
    cfg.observer.initialState=[reference.x(1)+.5;0;0;reference.y(1)-.4;0;0;reference.psi(1)+deg2rad(2)];
    cfg.measurement.inputInterpolation="zoh";
    cfg.measurement.timestampTolerance=0;
    saved=load(fullfile(sequenceFolder,'observer_positionOutage','report.mat'),'report');
    [weights,linearization,gains,stateTrace]=localDiagnostics(inputs,designs,cfg,saved.report);
    writetable(weights,fullfile(outputFolder,'event_information.csv'));
    writetable(linearization,fullfile(outputFolder,'local_linearization.csv'));
    writetable(gains,fullfile(outputFolder,'position_gains.csv'));
    writetable(stateTrace,fullfile(outputFolder,'baseline_state_trace.csv'));

    % A temporary source copy exposes bounded diagnostic substitutions. Its
    % returned certificate flag is explicitly false; no certificate is reused.
    scratch=string(tempname); mkdir(scratch);
    source=fileread(which('runImprovedVehicleObserver'));
    diagnosticSource=makeDiagnosticSource(source);
    writeText(fullfile(scratch,'runMncavDiagnosticObserver.m'),diagnosticSource);
    addpath(scratch,'-begin');
    cleanup=onCleanup(@() removeScratch(scratch));
    names=["baseline","perfectPose","zeroDelay","unitWeight", ...
        "hold120ms","basePositionGain","baseGainUnitWeight", ...
        "substeps4","noInvariant"];
    records=struct([]);
    traces=table();
    for index=1:numel(names)
        name=names(index); data=inputs; runCfg=cfg;
        runCfg.diagnostic=struct('substeps',1,'basePositionGain',false,'noInvariant',false);
        if name=="perfectPose"
            % Oracle ablation only; never enters the primary localization run.
            data.lidar.pose=interp1(reference.time,reference{:,2:4},data.lidar.timestamp,'linear','extrap');
        elseif name=="zeroDelay"
            data.lidar.arrivalTime=data.lidar.timestamp;
        elseif name=="unitWeight"
            runCfg.lidar.translationInformationScale=1e-8;
        elseif name=="hold120ms"
            runCfg.measurement.lidarMaximumAge=.12;
        elseif name=="basePositionGain"
            runCfg.diagnostic.basePositionGain=true;
        elseif name=="baseGainUnitWeight"
            runCfg.diagnostic.basePositionGain=true;
            runCfg.lidar.translationInformationScale=1e-8;
        elseif name=="substeps4"
            runCfg.diagnostic.substeps=4;
        elseif name=="noInvariant"
            runCfg.diagnostic.noInvariant=true;
        end
        record=struct('caseName',name,'completedTo61s',false,'failureMessage',"", ...
            'firstNonfiniteTime',NaN,'runSeconds',NaN,'outageRmseM',NaN, ...
            'outageMaximumM',NaN,'positionErrorAt44s',NaN,'positionErrorAt46s',NaN, ...
            'positionErrorAt48s',NaN,'maximumSpeedMps',NaN,'maximumAccelerationMps2',NaN, ...
            'baselineMaximumStateDifference',NaN);
        timer=tic;
        try
            estimate=runMncavDiagnosticObserver(prefix(data,61), ...
                designs.lateralDesign,designs.observerDesign,runCfg);
            record.completedTo61s=true;
        catch exception
            record.failureMessage=string(exception.message);
            token=regexp(exception.message,'DIAGNOSTIC_TIME=([0-9.]+)','tokens','once');
            if ~isempty(token), record.firstNonfiniteTime=str2double(token{1}); end
            % Retain a bounded prefix before the reported failing interval.
            last=48;
            if isfinite(record.firstNonfiniteTime),last=min(last,floor(record.firstNonfiniteTime)-1);end
            estimate=runMncavDiagnosticObserver(prefix(data,last), ...
                designs.lateralDesign,designs.observerDesign,runCfg);
        end
        record.runSeconds=toc(timer);
        t=estimate.time; state=estimate.onlineZ;
        ref=interp1(reference.time,reference{:,2:4},t,'linear');
        positionError=hypot(state(:,1)-ref(:,1),state(:,4)-ref(:,2));
        yawError=rad2deg(atan2(sin(state(:,7)-ref(:,3)),cos(state(:,7)-ref(:,3))));
        speed=hypot(state(:,2),state(:,5)); acceleration=hypot(state(:,3),state(:,6));
        outage=t>=40 & t<60;
        if record.completedTo61s
            record.outageRmseM=sqrt(mean(positionError(outage).^2));
            record.outageMaximumM=max(positionError(outage));
        end
        record.positionErrorAt44s=interp1(t,positionError,44);
        record.positionErrorAt46s=interp1(t,positionError,46);
        record.positionErrorAt48s=interp1(t,positionError,48);
        record.maximumSpeedMps=max(speed);
        record.maximumAccelerationMps2=max(acceleration);
        if name=="baseline"
            expected=saved.report.estimate.onlineZ(1:numel(t),:);
            record.baselineMaximumStateDifference=max(abs(state-expected),[],'all');
            assert(record.baselineMaximumStateDifference==0,'Diagnostic copy changed the baseline.');
        end
        records=[records;record]; %#ok<AGROW>
        rows=find(t>=39 & mod(round(t*100),10)==0);
        trace=table(repmat(name,numel(rows),1),t(rows),positionError(rows),yawError(rows), ...
            speed(rows),acceleration(rows), ...
            'VariableNames',{'caseName','time','positionErrorM','yawErrorDeg','speedMps','accelerationMps2'});
        traces=[traces;trace]; %#ok<AGROW>
        save(fullfile(outputFolder,name+'.mat'),'estimate','runCfg','record','-v7.3');
        writetable(struct2table(records),fullfile(outputFolder,'ablations.csv'));
        writetable(traces,fullfile(outputFolder,'error_traces.csv'));
        fprintf('%s: completed=%d, failure=%.6f s, error48=%.6g m\n', ...
            name,record.completedTo61s,record.firstNonfiniteTime,record.positionErrorAt48s);
    end
    results=struct('ablations',struct2table(records),'linearization',linearization);
    writeText(fullfile(outputFolder,'diagnostic_observer_source.m.txt'),diagnosticSource);
end

function data=prefix(data,last)
    keep=data.highRate.time<=last;
    for name=string(fieldnames(data.highRate)).'
        data.highRate.(name)=data.highRate.(name)(keep);
    end
end

function source=makeDiagnosticSource(source)
    source=replace(source,'function estimate = runImprovedVehicleObserver(', ...
        'function estimate = runMncavDiagnosticObserver(');
    before='    lidarGain = scaling * (design.P \ fusion.Cl.'');';
    after=sprintf('%s\n    if cfg.diagnostic.basePositionGain, lidarGain = scaling * design.K(:,1:2); end',before);
    assert(count(source,before)==1); source=replace(source,before,after);
    before='    derivative = channels.modelDerivative + baseCorrection + lidarCorrection + invariantCorrection;';
    after=sprintf('    if cfg.diagnostic.noInvariant, invariantCorrection(:)=0; end\n%s',before);
    assert(count(source,before)==1); source=replace(source,before,after);
    first=strfind(source,'    if method == "euler"');
    last=strfind(source,'function derivative = observerDerivative');
    block=source(first:last-1);
    block=replace(block,'    if method == "euler"',sprintf([ ...
        '    subdivisions=cfg.diagnostic.substeps;\n' ...
        '    stepTime=stepTime/subdivisions;\n' ...
        '    for substep=1:subdivisions\n' ...
        '    alpha0=(substep-1)/subdivisions;\n' ...
        '    try\n' ...
        '    if method == "euler"']));
    block=replace(block,'intervalIdx, 0.0,','intervalIdx, alpha0,');
    block=replace(block,'intervalIdx, 0.5,','intervalIdx, alpha0+0.5/subdivisions,');
    block=replace(block,'intervalIdx, 1.0,','intervalIdx, alpha0+1/subdivisions,');
    ending=sprintf('        "The improved observer produced a nonfinite state in interval %%d.", intervalIdx);\nend');
    newEnding=sprintf([ ...
        '        "The improved observer produced a nonfinite state in interval %%d.", intervalIdx);\n' ...
        '    catch exception\n' ...
        '        error(''MncavDiagnostic:Nonfinite'',''DIAGNOSTIC_TIME=%%.9f: %%s'', ...\n' ...
        '            highRate.time(intervalIdx)+substep*stepTime,exception.message);\n' ...
        '    end\n' ...
        '    state=stateNext;\n' ...
        '    end\nend']);
    assert(count(block,ending)==1); block=replace(block,ending,newEnding);
    source=[source(1:first-1),block,source(last:end)];
    before='    estimate.observer = struct("P", design.P, "K", design.K, "N", design.N, ...';
    after=sprintf('    design.certified=false; design.verification=struct(''diagnosticOnly'',true);\n%s',before);
    assert(count(source,before)==1); source=replace(source,before,after);
end

function [weights,linearization,gains,stateTrace]=localDiagnostics(inputs,designs,cfg,report)
    selected=find(inputs.lidar.timestamp>=40 & inputs.lidar.timestamp<49.57);
    values=zeros(numel(selected),8); weightPages=zeros(2,2,numel(selected)); headings=zeros(numel(selected),1);
    for k=1:numel(selected)
        j=selected(k); info=inputs.lidar.information(:,:,j);
        [weight,heading]=computeLidarInformationWeights(info,cfg);
        values(k,:)=[inputs.lidar.timestamp(j),eig(info(1:2,1:2)).',eig(weight).', ...
            heading,min(eig(info)),cond(info)];
        weightPages(:,:,k)=weight; headings(k)=heading;
    end
    weights=array2table(values,'VariableNames',{'time','informationWeak','informationStrong', ...
        'weightWeak','weightStrong','headingWeight','minimumFullInformationEigenvalue','fullInformationCondition'});
    scale=diag(cfg.observer.theta.^cfg.observer.scalingExponents);
    Cl=zeros(2,7);Cl(1,1)=1;Cl(2,4)=1;Cb=[Cl;0 0 0 0 0 0 1];
    L=scale*(designs.observerDesign.P\Cl.'); K=scale*designs.observerDesign.K;
    gains=array2table([(1:7).',L,K(:,1:2)], ...
        'VariableNames',{'state','lidarX','lidarY','baseX','baseY'});
    k=find(report.estimate.time==40); z=report.estimate.onlineZ(k,:).';
    q=report.estimate.trackAngleRate(k); beta=report.estimate.lateral.sideSlipAngle(k);
    A=zeros(7);A(1,2)=1;A(2,3)=1;A(4,5)=1;A(5,6)=1;
    A(3,2)=q^2;A(3,6)=-2*q;A(6,5)=q^2;A(6,3)=2*q;
    H=zeros(4,7);H(1,[2 5])=2*z([2 5]);
    H(2,[2 3 5 6])=z([3 2 6 5]);H(3,[2 3 5 6])=[z(6),-z(5),-z(3),z(2)];
    H(4,[2 5 7])=[-sin(z(7)+beta),cos(z(7)+beta),-z(5)*sin(z(7)+beta)-z(2)*cos(z(7)+beta)];
    Joff=A-scale*designs.observerDesign.N/cfg.observer.theta^3*H;
    W=mean(weightPages,3);wpsi=mean(headings);
    names=["actualWeight","unitWeight","baseGainActualWeight","baseGainUnitWeight"];
    rows=zeros(numel(names),3);
    for j=1:numel(names)
        gain=L;weight=W;
        if contains(names(j),'baseGain'),gain=K(:,1:2);end
        if contains(lower(names(j)),'unitweight'),weight=eye(2);end
        Jon=Joff-gain*weight*Cl-K*diag([0 0 wpsi])*Cb;
        rows(j,:)=[max(real(eig(Jon))),max(real(eig(Joff))), ...
            max(abs(eig(expm(Joff*.07)*expm(Jon*.03))))];
    end
    linearization=table(names.',rows(:,1),rows(:,2),rows(:,3), ...
        'VariableNames',{'caseName','continuousOnMaxRealEigenvalue','offMaxRealEigenvalue','period100msSpectralRadius'});
    idx=round([39 40 41 42 43 44 45 46 47 48 49 49.4]*100)+1;
    stateTrace=array2table([report.estimate.time(idx),report.online.positionErrorM(idx), ...
        report.estimate.onlineZ(idx,[2 3 5 6])], ...
        'VariableNames',{'time','positionErrorM','Vx','Ax','Vy','Ay'});
    terms=zeros(numel(idx),4);
    for j=1:numel(idx)
        sample=struct();k=idx(j);
        for name=["longitudinalSpeed","longitudinalAcceleration","lateralAcceleration","yawRate"]
            sample.(name)=inputs.highRate.(name)(k);
        end
        for name=["lateralVelocity","sideSlipAngle","sideSlipAngleRate"]
            sample.(name)=report.estimate.lateral.(name)(k);
        end
        state=report.estimate.onlineZ(k,:).';
        channels=evaluateImprovedObserverChannels(state,sample);
        correction=scale*designs.observerDesign.N/cfg.observer.theta^3*channels.invariantInnovation;
        terms(j,:)=[sample.longitudinalSpeed,hypot(state(2),state(5)),correction(5),norm(correction)];
    end
    stateTrace=[stateTrace,array2table(terms,'VariableNames',{'measuredLongitudinalSpeed', ...
        'estimatedSpeed','invariantVyDerivative','invariantCorrectionNorm'})];
end

function writeText(path,text)
    fid=fopen(path,'w');assert(fid>=0);cleanup=onCleanup(@() fclose(fid));
    fprintf(fid,'%s\n',text);
end

function removeScratch(path)
    rmpath(path);clear runMncavDiagnosticObserver;
    rmdir(path,'s');
end
