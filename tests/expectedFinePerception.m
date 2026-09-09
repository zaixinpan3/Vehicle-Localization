function expected = expectedFinePerception(dataset, frameIndex)
% expectedFinePerception: Immutable baseline plus reviewed pole-recovery cases.
% Original output evidence stays unchanged. These additions record the intended
% fine-detector correction; they are regression expectations, not ground truth.
    expected=loadPerceptionMaskReference(dataset,frameIndex);
    root=fileparts(fileparts(mfilename('fullpath')));
    changes=jsondecode(fileread(fullfile(root,'tests','reference','finePoleRecovery.json')));
    dataset=lower(string(dataset));
    if dataset=="mississippi", dataset="missisipi"; end
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.pole(entry.addedPoleIndices)=true;
        end
    end
end
