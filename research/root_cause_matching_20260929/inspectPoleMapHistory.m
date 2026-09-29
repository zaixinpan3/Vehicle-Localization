function inspectPoleMapHistory()
% inspectPoleMapHistory Diagnose view-dependent fine-map pole observations.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    a=load('output/mississippi_mapping_calibrated/feature_observations.mat','featureData');d=a.featureData;
    ref=load('output/root_cause_matching_20260929/frame806.mat','frameData');r=ref.frameData.reference;R=[cos(r(3)) -sin(r(3));sin(r(3)) cos(r(3))];
    at=find(string(d.featureNames)=="pole");rows=cell(0,7);
    for k=1:1170
        xyz=d.pointsByFeatureFrame{at,k};xy=(xyz(:,1:2)-r(1:2))*R;keep=sum((xy-[2.525 -4.394]).^2,2)<=1.3^2;
        if nnz(keep)<3,continue;end
        q=xy(keep,:);rows(end+1,:)={k,nnz(keep),mean(q(:,1)),mean(q(:,2)),std(q(:,1)),std(q(:,2)),mean(xyz(keep,3))}; %#ok<AGROW>
    end
    t=cell2table(rows,VariableNames={'frame','points','x','y','stdX','stdY','meanZ'});disp(t);writetable(t,fullfile(dest,'pole_map_view_history_806.csv'));
end
