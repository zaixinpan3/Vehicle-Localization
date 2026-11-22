function expected = expectedFinePerception(dataset, frameIndex)
% expectedFinePerception Load reviewed offline mask decisions for one frame.
    persistent fixture
    if isempty(fixture)
        fixture=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')), ...
            'reference','finePerceptionMaskChanges.json')));
    end
    if strcmpi(dataset,'Mississippi'),dataset="Missisipi";end
    base=loadPerceptionMaskReference(dataset,frameIndex);
    assert(base.revision==string(fixture.baseRevision),'perception:ReferenceRevision', ...
        'The mask changes and point-index reference must share a revision.');
    entry=fixture.entries(strcmpi({fixture.entries.dataset},dataset) & [fixture.entries.frameIndex]==frameIndex);
    changes=entry.changedPointIndices;
    masks=struct();
    for name=string(fieldnames(changes)).'
        mask=base.featureMasks.(name);
        indices=changes.(name);
        mask(indices)=~mask(indices);
        masks.(name)=mask;
    end
    expected=struct('featureMasks',masks);
end
