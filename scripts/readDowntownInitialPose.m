function packet=readDowntownInitialPose(datasetFolder,frameIndex,nativeTime)
% readDowntownInitialPose Export exactly one initialization pose then close CSV.
% Stop at the requested original acquisition; future pose rows are not read.
% This bootstrap helper is separate from the sensor-only tracking runtime.
    path=fullfile(datasetFolder,'mapping_poses.csv');f=fopen(path,'r');
    assert(f>=0,'VehicleLocalization:InitialReferenceMissing','Initial reference file is unavailable.');
    cleanup=onCleanup(@()fclose(f)); %#ok<NASGU>
    header=string(strsplit(fgetl(f),','));header=strip(header);
    fields=["frame_index","lidar_stamp_sec","pose_x_m","pose_y_m","pose_z_m","pose_qw","pose_qx","pose_qy","pose_qz"];
    [present,columns]=ismember(fields,header);assert(all(present),'VehicleLocalization:InitialReferenceSchema','Require canonical reference columns.');
    found=false;
    while ~feof(f)
        line=fgetl(f);if ~ischar(line),break;end
        tokens=strsplit(line,',','CollapseDelimiters',false);id=str2double(tokens{columns(1)});
        if id~=frameIndex,continue;end
        values=cellfun(@str2double,tokens(columns));
        assert(all(isfinite(values)) && abs(values(2)-nativeTime)<1e-7, ...
            'VehicleLocalization:InitialReferenceClock','Initial reference must match the exact first query clock.');
        row=array2table(values,'VariableNames',cellstr(fields));pose=poseSupport.poseRowToPlanarPose(row);found=true;break;
    end
    assert(found,'VehicleLocalization:InitialReferenceMissing','No reference exists for the first query; extrapolation is forbidden.');
    packet=struct('schemaVersion',1,'mode',"reference_pose_once",'frameIndex',frameIndex, ...
        'nativeTimeSeconds',nativeTime,'poseXYTheta',pose,'referencePoseCount',1, ...
        'referencePoint',"front LiDAR origin",'scope',"First query pose only; no velocity, acceleration or future reference pose");
end
