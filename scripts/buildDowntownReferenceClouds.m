function path=buildDowntownReferenceClouds(datasetFolder)
% buildDowntownReferenceClouds Pack measurement-only organized raw cloud slots.
% No mapping/reference pose is read. Missing native timing stays an empty slot.
    frames=readtable(fullfile(datasetFolder,'frames.csv'));n=height(frames);
    assert(isequal(frames.frame_index(:).',1:n),'Original raw frame IDs must be retained.');
    path=fullfile(datasetFolder,'pointClouds.mat');if isfile(path),return;end
    pointClouds=repmat(struct('x',single([]),'y',single([]),'z',single([]), ...
        'intensity',single([]),'pointTimeSeconds',[],'timestamp',NaN),1,n);
    for k=1:n
        if ~frames.available(k),continue;end
        f=load(fullfile(datasetFolder,'frames',sprintf('%06d.mat',k)));
        assert(abs(f.timestamp-frames.native_time_sec(k))<1e-7);
        pointClouds(k)=struct('x',f.x,'y',f.y,'z',f.z,'intensity',f.intensity, ...
            'pointTimeSeconds',f.pointTimeSeconds,'timestamp',f.timestamp);
    end
    save(path,'pointClouds','-v7.3');
end
