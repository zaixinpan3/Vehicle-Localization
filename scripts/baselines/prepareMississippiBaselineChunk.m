function prepareMississippiBaselineChunk(folder,first,last)
% Independent offline feature block; no gains or localization states stored.
    setupVehicleLocalization();addpath(fileparts(mfilename('fullpath')));maxNumCompThreads(1);
    frames=readtable(fullfile(folder,'frames.csv'));poses=readFramePoseTable(fullfile(folder,'poses.csv'),1:height(frames));
    names=perceptionConfig("Mississippi","offline");pc=perceptionConfig("Mississippi");tiltRotation=movingStartLidarTilt(folder,frames.stamp_sec,poses(1,:));
    points=cell(numel(names.featureNames),last-first+1);sources=cell(last-first+1,1);
    for j=first:last
        b=load(fullfile(folder,sprintf('frame_%04d.mat',j)));f=b.frame;
        xyz=[double(f.x(:)),double(f.y(:)),double(f.z(:))];range=double(f.range(:));good=all(isfinite(xyz),2) & range>1;
        assert(any(good) && prctile(abs(vecnorm(xyz(good,:),2,2)-range(good)),95)<1,'VehicleLocalization:BaselineRangeUnits','Stored range and XYZ must both be metres.');
        r=perceiveFrame(f,names);pc.coarseProbabilityCloud.projectionRotation=tiltRotation(:,:,j);sources{j-first+1}=perceiveCoarseProbabilityCloud(f,pc);
        xyz=xyz*names.frameCalibration.rotation.'+names.frameCalibration.translation;globalXYZ=registerPointsToGlobalFrame(xyz,poses(j,:));
        for c=1:numel(names.featureNames),mask=logical(r.featureMasks.(names.featureNames(c))(:)) & all(isfinite(globalXYZ),2);points{c,j-first+1}=globalXYZ(mask,:);end
        if mod(j-first+1,25)==0,fprintf('BLOCK %d-%d frame %d\n',first,last,j);end
    end
    configuration=names;save(fullfile(folder,sprintf('features_%04d_%04d.mat',first,last)),'points','sources','configuration','first','last','-v7.3');
end
