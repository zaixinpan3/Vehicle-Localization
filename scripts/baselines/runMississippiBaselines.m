function runMississippiBaselines(outputFolder,options)
% Fresh-gain, paired Mississippi comparison using current production observer.
% Arrays are temporary; only aggregate metrics, figures and provenance remain.
    arguments
        outputFolder string="output/mississippi_baselines_20261030"
        options.Drive double=1
        options.PrepareOnly logical=false
        options.TemporaryFolder string="/tmp/pdvl_mississippi_baselines"
    end
    root=setupVehicleLocalization();addpath(fullfile(root,'scripts','baselines'));maxNumCompThreads(2);
    external=fullfile(fileparts(root),'RobustVehicleLocalization','external');addpath(genpath(fullfile(external,'YALMIP')));addpath(genpath(fullfile(external,'sedumi')));
    if ~isfolder(outputFolder),mkdir(outputFolder);end
    [~,cfg]=synthesizeMncavInvariantDesign(mncavSmallOffsetObserverConfig());lc=lateralObserverConfig();ld=designLateralObserverGains(lc);
    [data,lateral,reference,meta]=inputs(options.Drive,options.TemporaryFolder,ld,lc,cfg);
    if options.PrepareOnly,return;end
    % Move the physical receiver point to the assumed design point using a
    % common sensor-only integrated heading. This is noisy preprocessing.
    measured=mncavFullObserverConfig();lever=measured.gnss.outputPoint.bodyOffset(:)-cfg.gnss.outputPoint.bodyOffset(:);
    t=data.highRate.time;yaw=reference(1,3)+cumtrapz(t,data.highRate.yawRate);
    data.gnss.position=data.gnss.position-[cos(yaw)*lever(1)-sin(yaw)*lever(2),sin(yaw)*lever(1)+cos(yaw)*lever(2)];
    v=[cos(reference(1,3)),-sin(reference(1,3));sin(reference(1,3)),cos(reference(1,3))]*[data.highRate.longitudinalSpeed(1);lateral.lateralVelocity(1)];
    cfg.initialState=[reference(1,1);v(1);0;reference(1,2);v(2);0;reference(1,3);0];
    meta.gnssPreprocessing="Measured-to-design lever correction using one initialized gyro-integrated heading; uncertainty remains a measurement disturbance";
    meta.globalCertificate=localizationSupport.verifyInvariantDesign(cfg.invariantDesign,cfg);meta.actualMapCertified=false;
    meta.gnssPositionFrames=nnz(data.gnss.valid);meta.receiverHeadingFrames=nnz(data.gnssHeading.valid);
    meta.queryFrames=numel(t);meta.queryIntervalMedianS=median(diff(t));meta.evaluationDurationS=t(end)-t(1);
    meta.mapComponents=data.lidarOverlap.map.components.numComponents;meta.velocityValidFrames=nnz(lateral.velocityValid);
    meta.currentScanOnly=false;meta.referenceSource="Corrected receiver reference";if options.Drive==2,meta.referenceSource="Original INSPVA; post-40s evaluation";end
    meta.initialization="One reference pose; measured velocity; acceleration and internal rate zero";
    save(fullfile(options.TemporaryFolder,"drive"+options.Drive+"_paired.mat"),'data','lateral','reference','cfg','meta','-v7.3');
    % Fixed grid calibrated on separate drive-2 prefix; caller can supply the
    % frozen choice through a compact parameter file, never fitted gains.
    tuningFile=fullfile(outputFolder,'ekf_tuning.json');
    assert(isfile(tuningFile),'VehicleLocalization:BaselineTuningRequired','Run tuneMississippiEkf on the declared calibration prefix before test replays.');
    tuning=jsondecode(fileread(tuningFile));
    if ~isfield(tuning,'gridVersion') || tuning.gridVersion~=4
        tuneMississippiEkf(outputFolder,options.TemporaryFolder);tuning=jsondecode(fileread(tuningFile));
    end
    meta.ekfTuning=tuning;
    t=t-t(1);rows={};
    methods=["proposed","overlap_ekf","overlap_iekf"];
    f=figure('Visible','off','Color','w');hold on;
    for method=methods
        timer=tic;
        if method=="proposed"
            e=runFullLocalizationObserver(data,cfg,LateralInputs=lateral);
        else
            iterations=1;chosen=tuning.ekf;
            if method=="overlap_iekf",iterations=5;chosen=tuning.iekf;end
            e=runInvariantEkfBaseline(data,lateral,cfg,LidarTemperature=chosen.temperature, ...
                ProcessScale=chosen.processScale,Iterations=iterations);
        end
        seconds=toc(timer);
        err=vecnorm(e.pose(:,1:2)-reference(:,1:2),2,2);
        delta=e.pose(:,3)-reference(:,3);angle=atan2(sin(delta),cos(delta));
        for window=["all","after_2s"]
            mask=true(size(t));if window=="after_2s",mask=t>=2;end
            complete=all(isfinite(e.pose(mask,:)),'all');
            m=[Inf,Inf,Inf,Inf];
            if complete,m=[rms(err(mask)),prctile(err(mask),95),max(err(mask)),rad2deg(rms(angle(mask)))];end
            rows(end+1,:)={options.Drive,"both",method,window,nnz(mask),m(1),m(2),m(3),m(4),complete,seconds,nnz(err(mask)>1)}; %#ok<AGROW>
        end
        plot(t,err,DisplayName=method);
    end
    grid on;ylabel('Position discrepancy (m)');xlabel('Time (s)');legend(Interpreter='none');
    tab=cell2table(rows,VariableNames={'drive','scenario','method','window','frames','rmseM','p95M','maximumM','headingRmseDeg','complete','runtimeS','framesAbove1m'});
    writetable(tab,fullfile(outputFolder,"drive"+options.Drive+"_metrics.csv"));
    set(findall(f,'Type','axes'),'Color','w','XColor','k','YColor','k','GridColor',[.7,.7,.7]);set(findall(f,'Type','text'),'Color','k');set(findall(f,'Type','legend'),'Color','w','TextColor','k');
    exportgraphics(f,fullfile(outputFolder,"drive"+options.Drive+"_comparison.png"),Resolution=160);close(f);
    fid=fopen(fullfile(outputFolder,"drive"+options.Drive+"_summary.json"),'w');fprintf(fid,'%s',jsonencode(meta,PrettyPrint=true));fclose(fid);
end
function [data,lateral,reference,meta]=inputs(drive,tmp,ld,lc,cfg)
    rawPath=fullfile(tmp,"drive"+drive+"_raw.mat");
    if isfile(rawPath)
        raw=load(rawPath,'data','lateral','reference','meta');
        if isfield(raw.data,'gnssHeading')
            data=raw.data;lateral=raw.lateral;reference=raw.reference;meta=raw.meta;
            if drive==2,[data,lateral,reference]=holdout(data,lateral,reference);end
            return
        end
    end
    if drive==1
        [status,headingJson]=system('uv run --offline --quiet --with numpy --with rosbags --with pyproj python scripts/readMncavGnssHeading.py');
        assert(status==0,'VehicleLocalization:MissingGnssHeading','Cannot read HEADING2: %s',headingJson);
        [p,meta]=prepareMncavMotionInputs("output/mncav_wheel_only_20260916/sensors");h=p.highRate;l=runLateralVelocityObserver(h,ld,lc);
        motion=struct('time',h.time,'longitudinalSpeed',h.longitudinalSpeed,'lateralVelocity',l.lateralVelocity,'yawRate',h.yawRate,'clockModelId',p.clockModelId,'velocityValid',l.velocityValid);
        cache=prepareMississippiLocalizationClouds(MotionInputs=motion);g=readtable('output/receiver_synchronized_inputs/bestpos.csv');native=readtable('output/receiver_reference/corrected_native_reference.csv');
        ref=interp1(native.time,[native.x,native.y,native.psi],cache.calls.timeSeconds,'linear');
    else
        folder=fullfile(tmp,'drive2');frameTable=readtable(fullfile(folder,'frames.csv'));poses=readFramePoseTable(fullfile(folder,'poses.csv'),1:height(frameTable));t=poses.receiver_time_sec;
        headingJson=fileread(fullfile(folder,'heading.json'));
        clock=loadReceiverClock(fullfile(folder,'inspva.csv'));frames=readtable(fullfile(folder,'frames.csv'));start=receiverClockTime(clock,frames.stamp_sec(1));finish=receiverClockTime(clock,frames.stamp_sec(end));
        [h,~,meta]=prepareWheelMotionInputs(fullfile(folder,'sensors'),mncavReplayConfig(),clock,start,finish,InitialSpeed=0);l=runLateralVelocityObserver(h,ld,lc);
        ref=zeros(height(poses),3);for j=1:height(poses),ref(j,:)=localizationSupport.poseRowToPlanarPose(poses(j,:));end
        names=perceptionConfig("Mississippi","offline");mc=featureMapBuildConfig();mc.logEnabled=false;mc.frameIndices=1:height(poses);
        observations=struct('featureNames',names.featureNames,'frameIndices',1:height(poses),'frameCalibration',names.frameCalibration,'pointsByFeatureFrame',{cell(numel(names.featureNames),height(poses))});sources=cell(height(poses),1);
        chunks=dir(fullfile(folder,'features_*.mat'));
        if ~isempty(chunks)
            covered=false(height(poses),1);
            for j=1:numel(chunks)
                block=load(fullfile(chunks(j).folder,chunks(j).name));assert(isequaln(block.configuration,names),'VehicleLocalization:BaselineCache','Feature configuration changed.');
                ids=block.first:block.last;assert(~any(covered(ids)));covered(ids)=true;observations.pointsByFeatureFrame(:,ids)=block.points;sources(ids)=block.sources;
            end
            assert(all(covered),'VehicleLocalization:BaselineCache','Complete feature coverage required.');
        else
            error('VehicleLocalization:BaselineCache','Prepare all feature blocks with prepareMississippiBaselineChunk first.');
        end
        assert(sum(cellfun(@(x)size(x,1),observations.pointsByFeatureFrame),'all')>0,'VehicleLocalization:EmptyBaselineMap','No mapping features extracted.');
        map=buildSlidingWindowMap(observations,mc);fixed=temporalMapToProbabilityCloud(map);cache=struct('fixed',fixed,'sources',{sources},'calls',table(t,VariableNames={'timeSeconds'}));g=readtable(fullfile(folder,'gnss.csv'));
    end
    n=height(cache.calls);I=nan(2,2,height(g));for k=find(logical(g.valid)).',I(:,:,k)=[g.informationXX(k),g.informationXY(k);g.informationXY(k),g.informationYY(k)];end
    data=struct('highRate',h,'gnss',struct('time',g.time,'position',[g.x,g.y],'information',I,'valid',logical(g.valid),'delay',0),'lidarOverlap',struct('time',cache.calls.timeSeconds,'map',cache.fixed,'sources',{cache.sources},'valid',true(n,1),'config',overlapGradientConfig()));
    data.gnssHeading=jsondecode(headingJson);
    [data,lateral]=localizationSupport.synchronizeLocalizationInputs(data,l,cfg);[ok,ids]=ismember(data.highRate.time,cache.calls.timeSeconds);assert(all(ok));reference=ref(ids,:);
    save(rawPath,'data','lateral','reference','meta','-v7.3');
    if drive==2,[data,lateral,reference]=holdout(data,lateral,reference);end
    meta.mapQueryProtocol="All-frame same-pass reference map; localizes measured-motion sources; not independent survey accuracy";
end
function out=slice(in,mask)
    out=in;n=numel(mask);for name=string(fieldnames(in)).',v=in.(name);if size(v,1)==n,out.(name)=v(mask,:);elseif ndims(v)==3 && size(v,3)==n,out.(name)=v(:,:,mask);end;end
end

function [data,lateral,reference]=holdout(data,lateral,reference)
    keep=data.highRate.time>=40;data.highRate=slice(data.highRate,keep);lateral=slice(lateral,keep);data.gnss=slice(data.gnss,keep);data.gnssHeading=slice(data.gnssHeading,keep);data.lidarOverlap.time=data.lidarOverlap.time(keep);data.lidarOverlap.sources=data.lidarOverlap.sources(keep);data.lidarOverlap.valid=data.lidarOverlap.valid(keep);reference=reference(keep,:);
end
