function names = validatePerceptionFeatureNames(names)
% validatePerceptionFeatureNames: Validate an ordered invocation channel list.
% Empty lists explicitly request no semantic channels. Do not silently drop
% unknown or repeated names, since that can hide a caller configuration error.
    assert(isstring(names) || iscellstr(names) || ischar(names), ...
        'perception:InvalidFeatureNames','featureNames must contain semantic names.');
    names=string(names);
    assert(isvector(names) || isempty(names), ...
        'perception:InvalidFeatureNames','featureNames must be a vector.');
    names=reshape(names,1,[]);
    supported=["curb","pole","facade","trafficSign"];
    assert(all(ismember(names,supported)), ...
        'perception:InvalidFeatureNames', ...
        'Supported features: curb, pole, facade, trafficSign.');
    assert(numel(unique(names))==numel(names), ...
        'perception:DuplicateFeatureNames','featureNames must not contain duplicate channels.');
end
