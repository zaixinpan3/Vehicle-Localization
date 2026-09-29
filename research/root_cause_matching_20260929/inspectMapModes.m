function inspectMapModes()
% inspectMapModes Expose nearby mode centers, scatter and prior mass.
    setupVehicleLocalization();dest=fileparts(mfilename('fullpath'));
    for frame=[178 806 854 893]
        d=load("output/root_cause_matching_20260929/frame"+frame+".mat");d=d.frameData;c=d.source.components;f=d.map.components;
        allPoint=d.current.semanticName~="curb";xy=d.current.mean(allPoint,:);near=any((d.fixedBodyMean(:,1)-xy(:,1).').^2+(d.fixedBodyMean(:,2)-xy(:,2).').^2<1.5^2,2) & f.semanticName~="curb";ids=find(near);rows=cell(numel(ids),10);
        R=[cos(d.reference(3)) -sin(d.reference(3));sin(d.reference(3)) cos(d.reference(3))];
        for j=1:numel(ids)
            i=ids(j);cv=R.'*f.covariance(:,:,i)*R;[v,e]=eig((cv+cv.')/2,'vector');[~,a]=max(e);meanZ=NaN;
            if isfield(f,'meanXYZ'),meanZ=f.meanXYZ(i,3);end
            rows(j,:)={i,f.semanticName(i),d.fixedBodyMean(i,1),d.fixedBodyMean(i,2),f.mixtureWeight(i),min(e),max(e),atan2(v(2,a),v(1,a)),meanZ,cv(1,2)};
        end
        t=cell2table(rows,VariableNames={'id','class','x','y','weight','minorVariance','majorVariance','majorAngle','meanZ','covarianceXY'});writetable(t,fullfile(dest,"map_modes_"+frame+".csv"));disp(frame);disp(t);
    end
end
