function [block, references] = shaftSpeedLoadFrames(root, dataset, frames)
% shaftSpeedLoadFrames: Read requested frames and earlier algorithm references.
    dataset = string(dataset); frames = double(frames(:).');
    if strcmpi(dataset,"Mississippi")
        filename = 'MissisipiPointClouds.mat'; label = 'full';
    elseif strcmpi(dataset,"Downtown")
        filename = 'downTownPointClouds.mat'; label = 'downtown';
    else
        error('Unsupported dataset: %s',dataset);
    end
    assert(~isempty(frames) && all(frames==floor(frames)) && all(frames>=1));
    source = matfile(fullfile(root,'data','raw',filename));
    if numel(frames)<3 || all(diff(frames)==frames(2)-frames(1))
        block = source.pointClouds(1,frames);
    else
        block = source.pointClouds(1,frames(1));
        for k=2:numel(frames), block(k)=source.pointClouds(1,frames(k)); end
    end
    previous = load(fullfile(root,'output','pillar_shaft_20260926', ...
        [label '_pipeline.mat']),'records','frames');
    references = cell(numel(frames),1);
    for k=1:numel(frames)
        j = find(previous.frames==frames(k),1);
        if ~isempty(j)
            references{k} = previous.records{j};
            references{k}.hasFineReference = strcmpi(dataset,"Downtown") || ...
                mod(frames(k)-1,10)==0;
        end
    end
end
