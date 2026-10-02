function matPath=buildCarlaPointCloudMat(datasetFolder)
% buildCarlaPointCloudMat Assemble prepared CARLA sweeps into pointClouds.mat.
% datasetFolder is the output of scripts/carla/prepareCarlaDataset.py. Each
% sweep becomes one element of the 1-by-N struct array pointClouds with
% single-precision x, y, z columns in the right-handed LiDAR frame (x forward,
% y left, z up), its CARLA semantic tag and instance index (evaluation labels
% only; perception reads x, y and z) and the simulation timestamp. Sweep k
% matches row k of poses.csv.
    arguments
        datasetFolder (1,1) string
    end
    poses=readtable(fullfile(datasetFolder,'poses.csv'),'TextType','string');
    n=height(poses);
    assert(isequal(poses.frame_index(:).',1:n),'Sweeps must be numbered 1..N in poses.csv.');
    template=struct('x',single([]),'y',single([]),'z',single([]),'semanticTag',uint8([]), ...
        'instance',uint32([]),'timestamp',0);
    pointClouds=repmat(template,1,n);
    for k=1:n
        stem=sprintf('%06d.bin',k-1);
        fid=fopen(fullfile(datasetFolder,'points',stem),'r','ieee-le');assert(fid>=0);
        xyz=fread(fid,[3 inf],'single=>single').';fclose(fid);
        count=size(xyz,1);
        assert(count==poses.points(k),'Point count mismatch at sweep %d.',k);
        fid=fopen(fullfile(datasetFolder,'labels',stem),'r','ieee-le');assert(fid>=0);
        tag=fread(fid,count,'uint8=>uint8');instance=fread(fid,count,'uint32=>uint32');fclose(fid);
        assert(numel(tag)==count && numel(instance)==count,'Label count mismatch at sweep %d.',k);
        pointClouds(k).x=xyz(:,1);pointClouds(k).y=xyz(:,2);pointClouds(k).z=xyz(:,3);
        pointClouds(k).semanticTag=tag;pointClouds(k).instance=instance;
        pointClouds(k).timestamp=poses.lidar_stamp_sec(k);
    end
    matPath=fullfile(datasetFolder,'pointClouds.mat');
    save(matPath,'pointClouds','-v7.3');
end
