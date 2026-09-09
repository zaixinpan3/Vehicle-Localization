function expected = expectedFinePerception(dataset, frameIndex)
% expectedFinePerception: Immutable outputs plus explicit fine pole and curb revisions.
% Original output evidence stays unchanged. Separate fixtures record intended
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
    changes=jsondecode(fileread(fullfile(root,'tests','reference','fineCurbGeometry.json')));
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.curb(:)=false;
            expected.featureMasks.curb(entry.curbIndices)=true;
        end
    end
end
