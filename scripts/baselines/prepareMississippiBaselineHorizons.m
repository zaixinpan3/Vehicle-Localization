function prepareMississippiBaselineHorizons(tmp)
% Build causal five-scan sources from aligned measured motion, never map poses.
    arguments
        tmp string="/tmp/pdvl_mississippi_baselines"
    end
    setupVehicleLocalization();path=fullfile(tmp,'drive2_raw.mat');b=load(path,'data','lateral','reference','meta');cfg=localizationSourceWindowConfig();
    if isfield(b.meta,'sourceWindowConfiguration'),assert(isequaln(b.meta.sourceWindowConfiguration,cfg));return;end
    h=b.data.highRate;l=b.lateral;t=h.time;
    [motion,segment]=integrateRecordedPlanarMotion(t,[h.longitudinalSpeed,l.lateralVelocity,h.yawRate],t,l.velocityValid);
    history=[];sources=cell(size(b.data.lidarOverlap.sources));
    for k=1:numel(t)
        [sources{k},history]=updateLocalizationSourceWindow(b.data.lidarOverlap.sources{k},t(k),motion(k,:),history,cfg,segment(k));
    end
    b.data.lidarOverlap.sources=sources;b.meta.sourceWindowConfiguration=cfg;
    b.meta.sourceMotionIntegration="Frame-aligned measured wheel/lateral/gyro motion; uninterrupted-valid segments; no map/reference pose";
    save(path,'-struct','b','-v7.3');fprintf('DRIVE2 causal horizons %d frames\n',numel(t));
end
