function [featureData,metadata] = reprojectSavedFeatureObservations(original,newPoses)
% reprojectSavedFeatureObservations Undo cached poses and apply new SE(3) poses.
% Feature selection and the already applied LiDAR calibration are preserved.
% Updating only the pose table would leave cached global points inconsistent.
    assert(isequal(double(original.frameIndices(:)),double(newPoses.frame_index(:))), ...
        'VehicleLocalization:FrameMismatch','New poses must cover the same ordered frames.');
    featureData=original;featureData.framePoseTable=newPoses;
    maximumRoundTrip=0;maximumLocalDifference=0;
    for k=1:height(newPoses)
        [oldR,oldT]=poseRowToRigidTransform(original.framePoseTable(k,:));
        [newR,newT]=poseRowToRigidTransform(newPoses(k,:));
        for j=1:numel(original.featureNames)
            points=double(original.pointsByFeatureFrame{j,k});
            if isempty(points),continue;end
            local=(points-oldT)*oldR;
            replacement=local*newR.'+newT;
            maximumRoundTrip=max(maximumRoundTrip,max(abs(local*oldR.'+oldT-points),[],'all'));
            maximumLocalDifference=max(maximumLocalDifference,max(abs((replacement-newT)*newR-local),[],'all'));
            featureData.pointsByFeatureFrame{j,k}=replacement;
        end
    end
    assert(maximumRoundTrip<1e-8 && maximumLocalDifference<1e-8,'VehicleLocalization:ReprojectionRoundTrip', ...
        'Cached transform could not be reversed consistently.');
    metadata=struct('maximumOriginalRoundTripM',maximumRoundTrip,'maximumLocalPointDifferenceM',maximumLocalDifference, ...
        'frames',height(newPoses),'sourcePointCounts',sum(original.counts,1), ...
        'method',"Full inverse old SE(3), followed by new SE(3); feature selection and LiDAR calibration unchanged");
    featureData.reprojection=metadata;
end
