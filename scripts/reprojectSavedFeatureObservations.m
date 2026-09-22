function [featureData,metadata] = reprojectSavedFeatureObservations(original,newPoses,options)
% reprojectSavedFeatureObservations Reproject unchanged raw feature selections.
% Undo both the cached world pose and its stored-to-reference calibration,
% then apply the requested calibration and new world pose exactly once.
% With no requested calibration, preserve the original calibration.
    arguments
        original (1,1) struct
        newPoses table
        options.FrameCalibration (1,1) struct=struct()
    end
    assert(isequal(double(original.frameIndices(:)),double(newPoses.frame_index(:))), ...
        'VehicleLocalization:FrameMismatch','New poses must cover the same ordered frames.');
    oldCalibration=lidarFrameCalibrationConfig();
    if isfield(original,'frameCalibration')
        oldCalibration=validateLidarFrameCalibration(original.frameCalibration);
    else
        assert(isempty(fieldnames(options.FrameCalibration)), ...
            'VehicleLocalization:MissingCalibration','An explicit source calibration is required to change calibration.');
    end
    newCalibration=oldCalibration;
    if ~isempty(fieldnames(options.FrameCalibration))
        newCalibration=validateLidarFrameCalibration(options.FrameCalibration);
    end
    featureData=original;featureData.framePoseTable=newPoses;featureData.frameCalibration=newCalibration;
    maximumRoundTrip=0;maximumLocalDifference=0;
    for k=1:height(newPoses)
        [oldR,oldT]=poseRowToRigidTransform(original.framePoseTable(k,:));
        [newR,newT]=poseRowToRigidTransform(newPoses(k,:));
        for j=1:numel(original.featureNames)
            points=double(original.pointsByFeatureFrame{j,k});
            if isempty(points),continue;end
            local=(points-oldT)*oldR;
            stored=(local-oldCalibration.translation)*oldCalibration.rotation;
            calibrated=stored*newCalibration.rotation.'+newCalibration.translation;
            replacement=calibrated*newR.'+newT;
            maximumRoundTrip=max(maximumRoundTrip,max(abs(local*oldR.'+oldT-points),[],'all'));
            recovered=((replacement-newT)*newR-newCalibration.translation)*newCalibration.rotation;
            maximumLocalDifference=max(maximumLocalDifference,max(abs(recovered-stored),[],'all'));
            featureData.pointsByFeatureFrame{j,k}=replacement;
        end
    end
    assert(maximumRoundTrip<1e-8 && maximumLocalDifference<1e-8,'VehicleLocalization:ReprojectionRoundTrip', ...
        'Cached transform could not be reversed consistently.');
    metadata=struct('maximumOriginalRoundTripM',maximumRoundTrip,'maximumLocalPointDifferenceM',maximumLocalDifference, ...
        'frames',height(newPoses),'sourcePointCounts',sum(original.counts,1), ...
        'oldCalibration',oldCalibration,'newCalibration',newCalibration, ...
        'method',"Inverse old world pose and old calibration; apply new calibration and new pose; raw feature selections preserved");
    featureData.reprojection=metadata;
end
