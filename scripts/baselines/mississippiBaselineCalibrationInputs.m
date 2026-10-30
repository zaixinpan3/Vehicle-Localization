function [d,l,ref,cfg]=mississippiBaselineCalibrationInputs(tmp)
% Shared calibration-only boundary; no evaluation-period reference enters tuning.
    prepareMississippiBaselineHorizons(tmp);
    b=load(fullfile(tmp,'drive2_raw.mat'),'data','lateral','reference');d=b.data;l=b.lateral;ref=b.reference;
    keep=d.highRate.time<40;
    for name=string(fieldnames(d.highRate)).',d.highRate.(name)=d.highRate.(name)(keep,:);end
    for name=string(fieldnames(l)).',l.(name)=l.(name)(keep,:);end
    d.gnss.time=d.gnss.time(keep);d.gnss.position=d.gnss.position(keep,:);d.gnss.information=d.gnss.information(:,:,keep);d.gnss.valid=d.gnss.valid(keep);
    d.lidarOverlap.time=d.lidarOverlap.time(keep);d.lidarOverlap.sources=d.lidarOverlap.sources(keep);d.lidarOverlap.valid=d.lidarOverlap.valid(keep);ref=ref(keep,:);
    cfg=mncavSmallOffsetObserverConfig();measured=mncavFullObserverConfig();lever=measured.gnss.outputPoint.bodyOffset(:)-cfg.gnss.outputPoint.bodyOffset(:);t=d.highRate.time;
    yaw=ref(1,3)+cumtrapz(t,d.highRate.yawRate);d.gnss.position=d.gnss.position-[cos(yaw)*lever(1)-sin(yaw)*lever(2),sin(yaw)*lever(1)+cos(yaw)*lever(2)];
    v=[cos(ref(1,3)),-sin(ref(1,3));sin(ref(1,3)),cos(ref(1,3))]*[d.highRate.longitudinalSpeed(1);l.lateralVelocity(1)];cfg.initialState=[ref(1,1);v(1);0;ref(1,2);v(2);0;ref(1,3);0];
end
