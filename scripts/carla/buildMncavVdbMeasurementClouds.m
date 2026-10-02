function path=buildMncavVdbMeasurementClouds(datasetFolder)
% buildMncavVdbMeasurementClouds Pack only measured XYZ and acquisition time.
% No pose table, semantic label or instance ID is read or retained.
    frames=readtable(fullfile(datasetFolder,'frames.csv'));n=height(frames);
    assert(isequal(frames.Properties.VariableNames,{'frame_index','lidar_stamp_sec','points'}) && ...
        isequal(frames.frame_index(:).',1:n),'VehicleLocalization:InvalidMeasurementFrames','Invalid measurement-only frame clock.');
    pointClouds=repmat(struct('x',single([]),'y',single([]),'z',single([]),'timestamp',0),1,n);
    for k=1:n
        f=fopen(fullfile(datasetFolder,'points',sprintf('%06d.bin',k-1)),'r','ieee-le');assert(f>=0);
        xyz=fread(f,[3 inf],'single=>single').';fclose(f);
        assert(size(xyz,1)==frames.points(k),'VehicleLocalization:InvalidPointCount','Point count differs.');
        pointClouds(k)=struct('x',xyz(:,1),'y',xyz(:,2),'z',xyz(:,3),'timestamp',frames.lidar_stamp_sec(k));
    end
    path=fullfile(datasetFolder,'pointClouds.mat');save(path,'pointClouds','-v7.3');
end
