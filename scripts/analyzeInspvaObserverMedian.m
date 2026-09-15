function report=analyzeInspvaObserverMedian(outputFolder)
% analyzeInspvaObserverMedian Diagnose the recorded RMSE/median tradeoff.
% All reference substitutions below are explicitly diagnostic oracle controls,
% not candidate localization results. Production observer/config is unchanged.
    arguments
        outputFolder (1,1) string="output/mncav_inspva_median_20260915"
    end
    setupVehicleLocalization();if ~isfolder(outputFolder),mkdir(outputFolder);end
    folder="output/mncav_inspva_observer_20260915";
    original=load(fullfile(folder,'experiment.mat'),'experiments','cfg');
    final=load(fullfile(folder,'motion_gap_experiment.mat'),'experiments');
    e=original.experiments{1};data=final.experiments{1}.data;actual=final.experiments{1}.estimate;
    lateral=e.lateralInput;cfg=original.cfg;t=data.highRate.time;uniform=e.uniformIndices;
    frames=readtable(fullfile(folder,'frame_errors.csv'),TextType="string");frames=frames(frames.mode=="per_frame_zero",:);
    full=frames.fullMeasurement==1;native=e.frameIndices(full);frameReference=e.frameReference(full,:);
    h=data.highRate;reference=e.reference;
    pva=readtable('data/raw/Missisipi/gnss/raw_data_2024-06-07-12-09-31_0_inspva.csv');
    nativeReference=readtable(fullfile(folder,'native_reference.csv'));assert(height(pva)==height(nativeReference));
    az=deg2rad(pva.azimuth_deg);
    bodyVelocity=[sin(az).*pva.east_velocity_mps+cos(az).*pva.north_velocity_mps, ...
        -cos(az).*pva.east_velocity_mps+sin(az).*pva.north_velocity_mps];
    referenceRate=gradient(nativeReference.psi,nativeReference.time);
    smoothBody=smoothdata(bodyVelocity,'movmean',.21,'SamplePoints',nativeReference.time);
    bodyAcceleration=[gradient(smoothBody(:,1),nativeReference.time)-referenceRate.*smoothBody(:,2), ...
        gradient(smoothBody(:,2),nativeReference.time)+referenceRate.*smoothBody(:,1)];
    oracleVelocity=interp1(nativeReference.time,bodyVelocity,t,'linear');
    oracleAcceleration=interp1(nativeReference.time,bodyAcceleration,t,'linear');
    oracleRate=interp1(nativeReference.time,referenceRate,t,'linear');
    measuredBody=smoothdata([h.longitudinalSpeed,lateral.lateralVelocity],'movmean',.21,'SamplePoints',t);
    kinematicAcceleration=[gradient(measuredBody(:,1),t)-h.yawRate.*measuredBody(:,2), ...
        gradient(measuredBody(:,2),t)+h.yawRate.*measuredBody(:,1)];
    names=["actual","reference_position_input","reference_heading_input","reference_heading_and_rate", ...
        "reference_body_velocity","reference_longitudinal_velocity","reference_lateral_velocity", ...
        "reference_body_acceleration","reference_velocity_acceleration", ...
        "reference_all_motion","zero_acceleration","kinematic_acceleration", ...
        "kp_2","kp_8","kp_12","kp_16","kp_24","kv_1","kv_12","kv_24","ka_6","ka_24"];
    estimates=cell(numel(names),1);rows=cell(0,12);
    for j=1:numel(names)
        candidate=data;lat=lateral;config=cfg;name=names(j);
        switch name
            case "reference_position_input",candidate.lidar.pose(:,1:2)=reference(:,1:2);
            case "reference_heading_input",candidate.lidar.pose(:,3)=reference(:,3);
            case "reference_heading_and_rate"
                candidate.lidar.pose(:,3)=reference(:,3);candidate.highRate.yawRate=oracleRate;
            case "reference_body_velocity"
                candidate.highRate.longitudinalSpeed=oracleVelocity(:,1);lat.lateralVelocity=oracleVelocity(:,2);
            case "reference_longitudinal_velocity"
                candidate.highRate.longitudinalSpeed=oracleVelocity(:,1);
            case "reference_lateral_velocity"
                lat.lateralVelocity=oracleVelocity(:,2);
            case "reference_body_acceleration"
                candidate.highRate.longitudinalAcceleration=oracleAcceleration(:,1);candidate.highRate.lateralAcceleration=oracleAcceleration(:,2);
            case "reference_velocity_acceleration"
                candidate.highRate.longitudinalSpeed=oracleVelocity(:,1);lat.lateralVelocity=oracleVelocity(:,2);
                candidate.highRate.longitudinalAcceleration=oracleAcceleration(:,1);candidate.highRate.lateralAcceleration=oracleAcceleration(:,2);
            case "reference_all_motion"
                candidate.lidar.pose(:,3)=reference(:,3);candidate.highRate.yawRate=oracleRate;
                candidate.highRate.longitudinalSpeed=oracleVelocity(:,1);lat.lateralVelocity=oracleVelocity(:,2);
                candidate.highRate.longitudinalAcceleration=oracleAcceleration(:,1);candidate.highRate.lateralAcceleration=oracleAcceleration(:,2);
                beta=unwrap(atan2(bodyVelocity(:,2),bodyVelocity(:,1)));
                valid=h.longitudinalSpeed>=.5;lat.sideSlipAngleRate(:)=0;
                betaRate=interp1(nativeReference.time,gradient(smoothdata(beta,'movmean',.21,'SamplePoints',nativeReference.time),nativeReference.time),t);
                lat.sideSlipAngleRate(valid)=betaRate(valid);
            case "zero_acceleration"
                candidate.highRate.longitudinalAcceleration(:)=0;candidate.highRate.lateralAcceleration(:)=0;
            case "kinematic_acceleration"
                candidate.highRate.longitudinalAcceleration=kinematicAcceleration(:,1);candidate.highRate.lateralAcceleration=kinematicAcceleration(:,2);
            otherwise
                if startsWith(name,"kp_"),config.gains(1)=str2double(extractAfter(name,"kp_"));end
                if startsWith(name,"kv_"),config.gains(2)=str2double(extractAfter(name,"kv_"));end
                if startsWith(name,"ka_"),config.gains(3)=str2double(extractAfter(name,"ka_"));end
        end
        estimates{j}=runMotionAidedVehicleObserver(candidate,lat,config);
        for population=["native_full","uniform_all"]
            if population=="native_full",ix=native;ref=frameReference;else,ix=uniform;ref=reference(ix,:);end
            rows(end+1,:)=[{name,startsWith(name,"reference_"),population,numel(ix)},num2cell(score(estimates{j}.pose(ix,:),ref))]; %#ok<AGROW>
        end
    end
    repeatDifference=max(abs(estimates{1}.z-actual.z),[],'all');assert(repeatDifference==0);
    metrics=cell2table(rows,VariableNames={'variant','referenceInjected','population','samples','rmseM','medianM','p95M','maximumM', ...
        'fractionAtMost5cm','fractionAtMost10cm','meanForwardErrorM','meanLeftErrorM'});
    raw=data.lidar.pose(native,:);observed=actual.pose(native,:);
    rawError=vecnorm(raw(:,1:2)-frameReference(:,1:2),2,2);actualError=vecnorm(observed(:,1:2)-frameReference(:,1:2),2,2);
    bins=[0,.05,.1,.2,inf];binRows=zeros(4,8);
    for j=1:4
        selected=rawError>=bins(j)&rawError<bins(j+1);
        binRows(j,:)=[bins(j),bins(j+1),nnz(selected),median(rawError(selected)),median(actualError(selected)), ...
            mean(actualError(selected)>rawError(selected)),sum(rawError(selected).^2),sum(actualError(selected).^2)];
    end
    distribution=array2table(binRows,VariableNames={'rawErrorLowerM','rawErrorUpperM','frames','rawMedianM','observerMedianM', ...
        'fractionWorse','rawSquaredErrorSum','observerSquaredErrorSum'});
    groups=["stationary","slow","moving","straight_moving","turning_moving","outside_long_gaps_and_1s_recovery"];
    masks={h.longitudinalSpeed(native)<.2,h.longitudinalSpeed(native)>=.2&h.longitudinalSpeed(native)<5, ...
        h.longitudinalSpeed(native)>=5,h.longitudinalSpeed(native)>=5&abs(h.yawRate(native))<.03, ...
        h.longitudinalSpeed(native)>=5&abs(h.yawRate(native))>=.03,true(numel(native),1)};
    times=t(native);gaps=diff(times);
    for j=find(gaps>.25).',masks{6}=masks{6}&~(times>=times(j)&times<=times(j+1)+1);end
    groupRows=cell(0,11);
    for j=1:numel(groups)
        for method=["raw","observer"]
            if method=="raw",pose=raw;else,pose=observed;end
            selected=masks{j};groupRows(end+1,:)=[{groups(j),method,nnz(selected)},num2cell(score(pose(selected,:),frameReference(selected,:)))]; %#ok<AGROW>
        end
    end
    grouped=cell2table(groupRows,VariableNames={'group','method','samples','rmseM','medianM','p95M','maximumM', ...
        'fractionAtMost5cm','fractionAtMost10cm','meanForwardErrorM','meanLeftErrorM'});
    [components,decomposition]=splitPositionError(data,actual,reference,cfg);
    positionOracle=estimates{names=="reference_position_input"};
    exactLidar=actual.position-positionOracle.position;exactMotion=positionOracle.position-reference(:,1:2);
    checks=struct('maximumLidarComponentVsOracleDifferenceM',max(vecnorm(exactLidar-components(:,1:2),2,2)), ...
        'maximumMotionComponentVsOracleDifferenceM',max(vecnorm(exactMotion-components(:,3:4),2,2)), ...
        'actualRepeatStateDifference',repeatDifference,'codeAnalyzer',checkcode(mfilename('fullpath'),'-config=factory','-id'));
    assert(checks.maximumLidarComponentVsOracleDifferenceM<1e-6 && checks.maximumMotionComponentVsOracleDifferenceM<2e-5);
    controlAudit=cell(numel(names),1);
    for j=1:numel(names)
        controlAudit{j}=struct('name',names(j),'gains',estimates{j}.observer.gains, ...
            'maximumCourseRate',estimates{j}.diagnostics.maximumTrackAngleRate, ...
            'rateEnvelopeSatisfied',estimates{j}.diagnostics.rateEnvelopeSatisfied);
    end
    checks.controls=vertcat(controlAudit{:});
    componentRows=cell(0,11);
    for method=["lidar_position_channel","motion_state_channel","component_sum"]
        switch method
            case "lidar_position_channel",error=components(:,1:2);
            case "motion_state_channel",error=components(:,3:4);
            otherwise,error=components(:,1:2)+components(:,3:4);
        end
        for population=["native_full","uniform_all"]
            if population=="native_full",ix=native;ref=frameReference;else,ix=uniform;ref=reference(ix,:);end
            pose=[reference(ix,1:2)+error(ix,:),reference(ix,3)];
            componentRows(end+1,:)=[{method,population,numel(ix)},num2cell(score(pose,ref))]; %#ok<AGROW>
        end
    end
    componentMetrics=cell2table(componentRows,VariableNames={'component','population','samples','rmseM','medianM','p95M','maximumM', ...
        'fractionAtMost5cm','fractionAtMost10cm','meanForwardErrorM','meanLeftErrorM'});
    c=cos(reference(:,3));s=sin(reference(:,3));
    navVelocity=[c.*oracleVelocity(:,1)-s.*oracleVelocity(:,2),s.*oracleVelocity(:,1)+c.*oracleVelocity(:,2)];
    velocityMismatch=actual.velocity-navVelocity;
    motionAudit=array2table([t,h.longitudinalSpeed,lateral.lateralVelocity,h.longitudinalAcceleration,h.lateralAcceleration, ...
        oracleVelocity,oracleAcceleration,velocityMismatch(:,1).*c+velocityMismatch(:,2).*s, ...
        -velocityMismatch(:,1).*s+velocityMismatch(:,2).*c,h.yawRate,oracleRate, ...
        atan2(sin(actual.headingUnwrapped-reference(:,3)),cos(actual.headingUnwrapped-reference(:,3))), ...
        interp1(nativeReference.time,double(nativeReference.ins_status==3),t,'previous')], ...
        VariableNames={'time','measuredVx','estimatedVy','measuredAx','measuredAy', ...
        'insVx','insVy','insDerivedAx','insDerivedAy','observerForwardVelocityError','observerLeftVelocityError', ...
        'measuredYawRate','insDerivedYawRate','observerHeadingErrorRad','insGood'});
    paired=table(frames.frame(full),t(native),rawError,actualError,components(native,1),components(native,2), ...
        components(native,3),components(native,4),h.longitudinalSpeed(native),h.yawRate(native), ...
        VariableNames={'frame','time','rawErrorM','observerErrorM','lidarComponentX','lidarComponentY','motionComponentX','motionComponentY','speedMps','yawRateRadps'});
    report=struct('metadata',struct('scope',"Diagnostic controls only; oracle substitutions are not localization accuracy claims", ...
        'baseFolder',folder,'mode',"per_frame_zero",'gains',cfg.gains,'productionModified',false, ...
        'derivativeSmoothingSeconds',.21,'repeatStateDifference',repeatDifference), ...
        'paired',struct('fractionWorse',mean(actualError>rawError),'fractionBetter',mean(actualError<rawError), ...
        'medianPairedDifferenceM',median(actualError-rawError),'differenceOfMediansM',median(actualError)-median(rawError)), ...
        'metrics',metrics,'distribution',distribution,'grouped',grouped,'decomposition',decomposition, ...
        'componentMetrics',componentMetrics,'validationChecks',checks);
    save(fullfile(outputFolder,'experiment.mat'),'report','estimates','components','motionAudit','paired','names','-v7.3');
    writetable(metrics,fullfile(outputFolder,'controls.csv'));writetable(distribution,fullfile(outputFolder,'raw_error_bins.csv'));
    writetable(grouped,fullfile(outputFolder,'groups.csv'));writetable(componentMetrics,fullfile(outputFolder,'components.csv'));
    writetable(motionAudit,fullfile(outputFolder,'motion_audit.csv'));writetable(paired,fullfile(outputFolder,'paired_frames.csv'));
    fid=fopen(fullfile(outputFolder,'summary.json'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    fidChecks=fopen(fullfile(outputFolder,'validation_checks.json'),'w');assert(fidChecks>=0);cleanupChecks=onCleanup(@()fclose(fidChecks));
    fprintf(fidChecks,'%s\n',jsonencode(checks,PrettyPrint=true));
    disp(metrics(metrics.population=="native_full",:));disp(componentMetrics);disp(distribution);disp(grouped);
end

function values=score(pose,reference)
    vector=pose(:,1:2)-reference(:,1:2);error=vecnorm(vector,2,2);c=cos(reference(:,3));s=sin(reference(:,3));
    values=[rms(error),median(error),prctile(error,95),max(error),mean(error<=.05),mean(error<=.1), ...
        mean(c.*vector(:,1)+s.*vector(:,2)),mean(-s.*vector(:,1)+c.*vector(:,2))];
end

function [components,audit]=splitPositionError(data,estimate,reference,cfg)
% e'=-K e + K n_lidar + (vhat-dp_reference/dt); split the two forced responses.
% Reference and saved velocity states are interpolated on the integration
% grid for this diagnostic. The reconstruction residual is reported explicitly.
    t=data.highRate.time;n=numel(t);K=zeros(2,2,n);weightRange=zeros(n,2);
    for k=1:n
        [~,~,~,W]=computeLidarInformationWeights(data.lidar.information(:,:,k),cfg);
        K(:,:,k)=cfg.gains(1)*W(1:2,1:2);weightRange(k,:)=eig(W(1:2,1:2)).';
    end
    noise=data.lidar.pose(:,1:2)-reference(:,1:2);velocity=estimate.velocity;
    state=[noise(1,:).';0;0];components=zeros(n,4);components(1,:)=state.';
    for k=2:n
        span=t(k)-t(k-1);pieces=max(1,ceil(span/cfg.maximumIntegrationStep-1e-10));dt=span/pieces;
        referenceVelocity=(reference(k,1:2)-reference(k-1,1:2)).'/span;
        for j=1:pieces
            a=(j-1)/pieces;b=j/pieces;c=(a+b)/2;
            d1=rhs(state,a);d2=rhs(state+dt*d1/2,c);d3=rhs(state+dt*d2/2,c);d4=rhs(state+dt*d3,b);
            state=state+dt*(d1+2*d2+2*d3+d4)/6;
        end
        components(k,:)=state.';
    end
    actual=estimate.position-reference(:,1:2);total=components(:,1:2)+components(:,3:4);
    residual=max(vecnorm(total-actual,2,2));assert(residual<2e-4,'Position decomposition must reproduce the original trajectory.');
    audit=struct('maximumPositionReconstructionResidualM',residual,'minimumPositionWeight',min(weightRange,[],'all'), ...
        'maximumPositionWeight',max(weightRange,[],'all'),'medianWeightEigenvalues',median(weightRange), ...
        'interpretation',"Vector contributions add; medians do not. Motion component includes all inconsistency with the reference trajectory derivative.");
    function derivative=rhs(x,fraction)
        A=(1-fraction)*K(:,:,k-1)+fraction*K(:,:,k);
        lidarNoise=((1-fraction)*noise(k-1,:)+fraction*noise(k,:)).';
        mismatch=((1-fraction)*velocity(k-1,:)+fraction*velocity(k,:)).'-referenceVelocity;
        derivative=[A*(lidarNoise-x(1:2));mismatch-A*x(3:4)];
    end
end
