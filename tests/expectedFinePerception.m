function expected = expectedFinePerception(dataset, frameIndex)
% expectedFinePerception: Immutable outputs plus explicit fine pole and curb revisions.
% Original output evidence stays unchanged. Separate fixtures record intended
% fine-detector correction; they are regression expectations, not ground truth.
    expected=loadPerceptionMaskReference(dataset,frameIndex);
    % Preserve the original evidence while comparing only retained channels.
    expected.featureMasks=rmfield(expected.featureMasks,'roadMarking');
    root=fileparts(fileparts(mfilename('fullpath')));
    changes=jsondecode(fileread(fullfile(root,'tests','reference','finePoleRecovery.json')));
    dataset=lower(string(dataset));
    if dataset=="mississippi", dataset="missisipi"; end
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.pole(entry.addedPoleIndices)=true;
        end
    end
    changes=jsondecode(fileread(fullfile(root,'tests','reference','finePoleRejection.json')));
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.pole(entry.removedPoleIndices)=false;
        end
    end
    changes=jsondecode(fileread(fullfile(root,'tests','reference','finePoleShaftCompletion.json')));
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.pole(entry.removedPoleIndices)=false;
            expected.featureMasks.pole(entry.addedPoleIndices)=true;
        end
    end
    changes=jsondecode(fileread(fullfile(root,'tests','reference','finePoleBoundaryRecovery.json')));
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.pole(entry.removedPoleIndices)=false;
            expected.featureMasks.pole(entry.addedPoleIndices)=true;
        end
    end
    changes=jsondecode(fileread(fullfile(root,'tests','reference','trafficSignThreshold.json')));
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.trafficSign(entry.removedTrafficSignIndices)=false;
        end
    end
    changes=jsondecode(fileread(fullfile(root,'tests','reference','fineCurbGeometry.json')));
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.curb(:)=false;
            expected.featureMasks.curb(entry.curbIndices)=true;
        end
    end
    changes=jsondecode(fileread(fullfile(root,'tests','reference','fineCurbRevision.json')));
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.curb(entry.removedCurbIndices)=false;
            expected.featureMasks.curb(entry.addedCurbIndices)=true;
        end
    end
    changes=jsondecode(fileread(fullfile(root,'tests','reference','fineCurbGuidedExtension.json')));
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.curb(entry.removedCurbIndices)=false;
            expected.featureMasks.curb(entry.addedCurbIndices)=true;
        end
    end

    changes=jsondecode(fileread(fullfile(root,'tests','reference','finePoleCompactSupport.json')));
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.pole(entry.removedPoleIndices)=false;
            expected.featureMasks.pole(entry.addedPoleIndices)=true;
        end
    end
    changes=jsondecode(fileread(fullfile(root,'tests','reference','fineCurbNormalConsistency.json')));
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.curb(entry.removedCurbIndices)=false;
            expected.featureMasks.curb(entry.addedCurbIndices)=true;
        end
    end

    changes=jsondecode(fileread(fullfile(root,'tests','reference','finePoleSupportBoundary.json')));
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.pole(entry.removedPoleIndices)=false;
            expected.featureMasks.pole(entry.addedPoleIndices)=true;
        end
    end
    changes=jsondecode(fileread(fullfile(root,'tests','reference','fineCurbLocalCompetition.json')));
    for entry=changes.entries(:).'
        if lower(string(entry.dataset))==dataset && entry.frameIndex==frameIndex
            expected.featureMasks.curb(entry.removedCurbIndices)=false;
            expected.featureMasks.curb(entry.addedCurbIndices)=true;
        end
    end
end
